# DeepStream Service Maker API

DeepStream **Service Maker** (SM) is a higher-level C++ interface over GStreamer/DeepStream. It reduces boilerplate for building pipelines and gives clean hook points to read **pixels + metadata**—while keeping the underlying DeepStream semantics (buffers, batching, `NvDs*` metadata) intact.

## Pipeline Construction

### Basic Pipeline API (minimal example)

```cpp
#include "pipeline.hpp"
#include <iostream>

using namespace deepstream;

#define CONFIG_FILE_PATH "/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_infer_primary.yml"

int main(int argc, char* argv[]) {
  try {
    // argv[1] = "file:///.../sample_720p.h264"
    Pipeline pipeline("sample-pipeline");
    pipeline
      .add("nvurisrcbin",   "src",  "uri", argv[1])
      .add("nvstreammux",   "mux",  "batch-size", 1, "width", 1280, "height", 720)
      .add("nvinferbin",    "infer","config-file-path", CONFIG_FILE_PATH)
      .add("nvosdbin",      "osd")
      .add("nveglglessink", "sink");

    // Link src→mux; request pad only on mux (sink_%u)
    pipeline.link({"src","mux"}, {"", "sink_%u"})
            .link("mux","infer","osd","sink");

    pipeline.start().wait();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}
```

**Notes**

* Namespace and headers follow SM conventions: `using namespace deepstream;` and `"pipeline.hpp"`.
* OSD element in SM’s factory naming is `nvosdbin` (the underlying plugin is `nvdsosd`).
* Prefer the YAML `config_infer_primary.yml` for `nvinferbin`.

## Data Inspection (in-path)

SM exposes **Buffer Probes** for lightweight work on the hot path. Prefer **metadata** probes for speed and safety.

### Probe interfaces (C++)

* `BufferProbe::IBatchMetadataObserver` – read-only batch metadata
* `BufferProbe::IBatchMetadataOperator` – read/write batch metadata
* `BufferProbe::IBufferObserver` – read-only buffer view
* `BufferProbe::IBufferOperator` – read/write buffer payload (use sparingly)

### Read-only metadata probe (recommended)

```cpp
#include "pipeline.hpp"
#include "buffer_probe.hpp"
#include <iostream>

using namespace deepstream;

class DetectionPrinter : public BufferProbe::IBatchMetadataObserver {
public:
  probeReturn handleData(BufferProbe& /*probe*/, const BatchMetadata& batch) override {
    batch.iterate([](const FrameMetadata& frame) {
      const int   sid = frame.sourceId();
      const auto  pts = frame.bufPts();      // ns
      int count = 0;

      frame.iterate([&](const ObjectMetadata& obj) {
        // Access classId, confidence, bbox, track id, etc.
        // Example: just count objects
        (void)obj;
        ++count;
      });

      std::cout << "source_id=" << sid
                << " pts=" << pts
                << " objects=" << count << "\n";
    });
    return probeReturn::Probe_Ok;
  }
};

int main(int argc, char* argv[]) {
  Pipeline p("probe-pipeline");
  // ... add/link like in the minimal example ...
  p.attach("infer", new BufferProbe("post-infer", new DetectionPrinter));
  p.start().wait();
}
```

**Why metadata probes?** They avoid CPU/GPU copies and keep the pipeline fast. Use them to read `BatchMetadata → FrameMetadata → ObjectMetadata` and attach small annotations if needed.

### Buffer payload probe (rare; keep tiny)

```cpp
class TouchBuffer : public BufferProbe::IBufferOperator {
public:
  probeReturn handleData(BufferProbe& /*probe*/, Buffer& buf) override {
    // If you must inspect pixels on-path, keep it minimal.
    VideoBuffer vbuf = buf;  // cast to video view if available
    vbuf.read([&](const void* data, size_t nbytes) -> size_t {
      (void)data; (void)nbytes;  // e.g., quick checksum or header peek
      return nbytes;
    });
    return probeReturn::Probe_Ok;
  }
};
```

## Heavy Processing (off-path)

Use **DataReceiver** at an element that emits signals (typically `appsink::new-sample`). This keeps heavy I/O, serialization, and conversions **off the hot path**.

### Graph tail (CPU handoff)

Ensure final caps drop GPU memory if you want CPU frames:

```cpp
// ... earlier elements ...
.add("nvvideoconvert", "cvt")
.add("capsfilter",     "caps", "caps", "video/x-raw,format=BGR") // CPU BGR
.add("appsink",        "sink", "emit-signals", true, "sync", false);
```

### DataReceiver consumer

```cpp
#include "pipeline.hpp"
#include "data_receiver.hpp"
#include <opencv2/imgcodecs.hpp>

using namespace deepstream;

class ExportConsumer : public DataReceiver::IDataConsumer {
public:
  int consume(DataReceiver& /*rx*/, Buffer buffer) override {
    // 1) Metadata (if present)
    if (buffer.hasBatchMetadata()) {
      const BatchMetadata& batch = buffer.getBatchMetadata();
      batch.iterate([](const FrameMetadata& f) {
        // Walk objects, serialize dets/tracks, etc.
        (void)f;
      });
    }

    // 2) Pixels (CPU BGR negotiated upstream)
    VideoBuffer vbuf = buffer;
    const int w = vbuf.width();
    const int h = vbuf.height();

    vbuf.read([&](const void* data, size_t size) -> size_t {
      // Assuming BGR8 tightly packed (verify stride in your caps if needed)
      cv::Mat bgr(h, w, CV_8UC3, const_cast<void*>(data));
      cv::imwrite("frame.jpg", bgr);  // example: heavy I/O off-path
      return size;
    });

    return 0; // success
  }
};

int main(int argc, char* argv[]) {
  Pipeline p("receiver-pipeline");
  // ... add/link, ending with "sink" appsink as above ...
  p.attach("sink", new DataReceiver("new-sample", new ExportConsumer));
  p.start().wait();
}
```

**Tips**

* `emit-signals=true` is required on `appsink` to fire `new-sample`.
* Do ROS 2 publishing, file/network export, JSON packing, etc. here.
* If you keep frames on GPU (no CPU caps), design your consumer accordingly.

## YAML Configuration

### Declarative pipeline (correct schema)

```yaml
deepstream:
  nodes:
    - type: nvurisrcbin
      name: src
      properties:
        uri: "file:///opt/nvidia/deepstream/deepstream/samples/streams/sample_720p.h264"

    - type: nvstreammux
      name: mux
      properties:
        batch-size: 1
        width: 1280
        height: 720

    - type: nvinferbin
      name: infer
      properties:
        config-file-path: /opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_infer_primary.yml

    - type: nvosdbin
      name: osd

    - type: nveglglessink
      name: sink

  edges:
    src: mux
    mux: infer
    infer: osd
    osd: sink
```

### Hybrid: YAML + C++ hooks

```cpp
#include "pipeline.hpp"
#include "buffer_probe.hpp"
#include "data_receiver.hpp"

using namespace deepstream;

int main() {
  Pipeline p = Pipeline::fromYaml("pipeline.yaml");

  p.attach("infer", new BufferProbe("post-infer", new DetectionPrinter));
  p.attach("sink",  new DataReceiver("new-sample", new ExportConsumer));

  p.start().wait();
}
```

## Integration Points & Best Practices

### Where to hook

| Stage              | What you get                                 | Typical use                     |
| ------------------ | -------------------------------------------- | ------------------------------- |
| After `nvinferbin` | Detections (`ObjectMetadata`)                | Real-time det logic, counters   |
| After `nvtracker`  | Detections + track IDs                       | Tracking/persistence analytics  |
| After `nvosdbin`   | Overlays drawn; metadata intact              | Final QA/telemetry              |
| At `appsink`       | Pixels (CPU or GPU) + full metadata off-path | ROS bridge, archival, streaming |

### Metadata classes (naming)

* `BatchMetadata` → frames
* `FrameMetadata` → per-frame info (`sourceId`, `bufPts`, etc.)
* `ObjectMetadata` → per-object det/track (classId, confidence, bbox, objectId)
* (Also available when produced: `ClassifierMetadata`, `DisplayMetadata`, `TensorOutputUserMetadata`, …)

### Performance

1. **Keep probes light** (log, small counters, tiny annotations).
2. **Do heavy work in DataReceiver** (appsink), not on the hot path.
3. **CPU vs GPU explicitly**: control with caps at the tail (`video/x-raw,format=BGR` for CPU; keep `memory:NVMM` if you want GPU).
4. **Batching**: `nvstreammux` batch semantics and identity via `FrameMetadata.sourceId + bufPts` remain your sync keys.

### Debugging

* You can still attach **raw GStreamer pad probes** to elements created by SM for exotic event/query timing or caps debugging.
* Validate caps at the tail (format, stride) to match your consumer assumptions.

### Lifecycle control

```cpp
pipeline.start();     // start streaming
pipeline.wait();      // block until EOS or stop
pipeline.stop();      // stop and teardown

// Advanced multi-pipeline bring-up:
pipeline.prepare();
pipeline.activate();
pipeline.wait();
```

## Summary

* Build pipelines declaratively with **Pipeline API** (or YAML).
* Use **metadata probes** for fast, in-path inspection.
* Use **DataReceiver** at **appsink** for off-path heavy tasks (export, ROS, storage).
* Keep the classic DeepStream mental model: **buffers carry pixels + `NvDs` metadata**; batching and `sourceId + bufPts` drive synchronization.

## References

- [NVIDIA DeepStream Service Maker Documentation](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_service_maker_cpp.html)
