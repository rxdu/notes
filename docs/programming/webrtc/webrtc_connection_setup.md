# WebRTC Connection Setup

## Conceptual Layering

In WebRTC, establishing a real-time connection between peers involves multiple coordinated steps across different protocol layers. Understanding these layers helps clarify how media and data flow securely and efficiently from one device to another.

You may think of the layers as follows:

* SDP: Configuration Layer
    * Describes media capabilities, codec preferences, transport parameters (ICE, DTLS), and negotiation state.
* ICE: Connectivity Layer
    * Discovers possible network paths (host, STUN, TURN), performs connectivity checks, and selects a working route.
* DTLS: Encryption Layer
    * Establishes a secure connection over the selected ICE path, negotiating keys for media and data encryption.
* SRTP / SCTP: Transport Layers
    * SRTP: Securely carries audio/video streams.
    * SCTP (over DTLS): Transports data channels with reliability and ordering options.

> SDP configures the session, ICE connects the peers, DTLS secures the transport, and SRTP/SCTP transmit the media and data.

## Peer Connection Components

The `RTCPeerConnection` object (in both browser/WebRTC native C++ API) encapsulates the full stack of components required to establish and manage a secure real-time connection. These components work together to handle signaling, connectivity, encryption, and media/data transmission.

#### 1. ICE Agent

* Gathers local ICE candidates (host, STUN, TURN)
* Performs STUN-based connectivity checks with the remote peer
* Selects and manages the most efficient and stable candidate pair

#### 2. DTLS Transport

* Manages DTLS certificates and handshakes
* Establishes a secure transport channel over the selected ICE candidate pair
* Derives session keys for SRTP (media) and SCTP (data)

#### 3. SRTP/SCTP Stack

* SRTP: Encrypts and decrypts audio/video RTP streams using keys from DTLS
* SCTP (over DTLS): Transports WebRTC data channels with reliability, ordering, and stream multiplexing

#### 4. Media Transceivers/Tracks

* Manages the sending and receiving of audio/video tracks
* Controls media direction (sendrecv, recvonly, inactive, etc.)
* Supports dynamic track replacement (e.g., switching cameras)

#### 5. Signaling State Machine

* Tracks the signaling flow (offer/answer negotiation)
* Manages state transitions (stable, have-local-offer, etc.)
* Triggers onnegotiationneeded when renegotiation is required
* Supports rollback in case of offer/answer conflicts or errors

#### Visual Timeline

```text
Time →
 ┌────────────┐                                   ┌──────────────┐
 │   Caller   │                                   │   Answerer   │
 └────────────┘                                   └──────────────┘
     |                                                 |
     | CreateOffer()                                   |
     |────────────────────────────────────────────────>|
     |                                                 |
     | SetLocalDescription(offer)                      |
     | (starts local ICE gathering)                    |
     |  ↓                                              |
     |  Emit local ICE candidates (trickle)            |
     |  ↓                                              |
     |─────────────────────[ Offer via Signaling ]────>|
     |                                                 |
     |                                    SetRemoteDescription(offer)
     |                                    CreateAnswer()
     |                                    SetLocalDescription(answer)
     |                                    (starts ICE gathering) 
     |                                    ↓
     |                                    Emit answerer ICE candidates
     |<────────────────────[ Answer via Signaling ]────|
     |                                                 |
     | SetRemoteDescription(answer)                    |
     |                                                 |
     |<─────────[ ICE candidates (from answerer) ]─────|
     |──────────[ ICE candidates (to answerer) ]──────>|
     |                                                 |
     |        Both sides perform ICE connectivity checks
     |       (STUN binding requests, find valid pair)  |
     |                                                 |
     |        ICE state: checking → connected          |
     |                                                 |
     |───────────── DTLS Handshake Begins ─────────────|
     |                                                 |
     |    Secure transport established (SRTP/SCTP)     |
     |                                                 |
     |<──── onTrack / onDataChannel / onConnection ───>|
     |         Media and Data Channels Active          |

```


## SDP and ICE Roles

Both SDP and ICE are exchanged during WebRTC signaling to let peers agree on media format and how to reach each other.

* SDP (Session Description Protocol) defines the **what** (codecs, media, etc.).
* ICE (Interactive Connectivity Establishment) defines the **how** (network paths for connectivity).

The SDP offer/answer model negotiates how peers want to communicate (media formats, ICE credentials, DTLS fingerprints, etc.), while the ICE process actually establishes the network path between them. Once ICE finds a valid candidate pair, DTLS sets up a secure transport, and SRTP/SCTP runs on top for media and data.

Negotiation is triggered mostly by signaling state changes that affect the SDP. **ICE restart is the only ICE-related action that requires renegotiation.**

| Event                                                        | Triggers Negotiation?     | Why                                      |
| ------------------------------------------------------------ | ------------------------- | ---------------------------------------- |
| `addTrack()`                                                 | ✅ Yes                     | Adds media to SDP                        |
| `removeTrack()`                                              | ✅ Yes                     | Changes media config                     |
| `addTransceiver()`                                           | ✅ Yes                     | Adds m-line in SDP                       |
| `setCodecPreferences()`                                      | ✅ Yes                     | Modifies offer capabilities              |
| Changing direction (`sendonly`, `recvonly`)                  | ✅ Yes                     | Affects media flow                       |
| Updating `RTCRtpSender` parameters (e.g., `setParameters()`) | ❌ No                      | Local change only                        |
| **ICE candidate discovered**                                 | ❌ No                      | Doesn’t affect media description         |
| **ICE restart** (`iceRestart: true`)                         | ✅ Yes                     | Special case — described below           |
| ICE failure/reconnection                                     | ❌ No (handled internally) | Unless manually restarted                |
| DTLS transport failure                                       | ❌ No (handled internally) | App can react, but no auto-renegotiation |

## SDP Offer/Answer Model

The SDP (Session Description Protocol) offer/answer model is a mechanism used in WebRTC to negotiate media parameters (such as codecs, IP addresses, ports, etc.) between two peers.

#### 1. Creating an Offer (caller)
   
   * The caller generates an SDP offer using the CreateOffer method of the webrtc::PeerConnectionInterface object.
   * Once the offer is created, it is set as the local description using the SetLocalDescription method.

#### 2. Sending the Offer (caller)

   * The SDP offer is sent to the other peer (the answerer) via a signaling server (e.g., WebSocket, SIP, etc.).

#### 3. Receiving the Offer (answerer)

   * The answerer receives the SDP offer and sets it as their remote description using the SetRemoteDescription method.

#### 4. Creating an Answer (answerer)

   * The answerer generates an SDP answer in response to the received offer using the CreateAnswer method of the webrtc::PeerConnectionInterface object.
   * Once the answer is created, it is set as the local description using the SetLocalDescription method.
  
#### 5. Sending the Answer (answerer)

   * The SDP answer is sent back to the caller via the signaling server.

#### 6. Receiving the Answer (caller)

   * The caller receives the SDP answer and sets it as their remote description using the SetRemoteDescription method.

## ICE Candidate Exchange

In parallel with the offer/answer exchange, both peers gather ICE (Interactive Connectivity Establishment) candidates. These candidates represent potential network paths (IP addresses and ports) that can be used to establish the media connection.

ICE candidates are sent to the remote peer via the signaling server and added using the AddIceCandidate method.

In Trickle ICE, a peer knows that all ICE candidates have been sent when it receives a special "end-of-candidates" signal. This can happen in two ways:

* ICE Candidate Event with null
* Signaling a Final Candidate (SDP or JSON)

Without this final signal:

* The ICE agent may wait indefinitely for more candidates.
* The connection state may stall in "checking" or "connecting".

```text
[Gathering ICE candidates...]
   ↓
onicecandidate → send each candidate
   ↓
onicecandidate → event.candidate === null
   ↓
Send "end-of-candidates" marker
   ↓
Remote peer: addIceCandidate(null)
   ↓
ICE completes (if a valid candidate pair found)
```

## State Transitions in WebRTC

The RTCPeerConnection object maintains a signaling state to track the progress of the offer/answer exchange:

* stable: Initial state, no ongoing SDP exchange.
* have-local-offer: A local SDP offer has been created and set by the local peer.
* have-remote-offer: A remote SDP offer has been received and set by the local peer.
* have-local-pranswer: A local SDP provisional answer has been created and set in response to a remote offer.
* have-remote-pranswer: A remote SDP provisional answer has been received in response to a local offer.
* closed: The connection is closed.
  
When the signaling state changes to **stable**, it indicates that:

* Both peers have successfully negotiated the SDP.
* The RTCPeerConnection is ready to proceed with media exchange.
* No pending offer/answer exchange is ongoing.
  
The state follows the following patterns before reaching stable state:

* The transition from stable to have-local-offer happens when an SDP offer is created and set as the local description by the caller.
* The transition from stable to have-remote-offer happens when an SDP offer is received and set as the remote description by the answerer.
* The transition back to stable happens when the caller sets the remote description to the received answer or when the answerer sets the local description to the created answer.

