# SDP Negotiation

The SDP (Session Description Protocol) offer/answer model is a mechanism used in WebRTC to negotiate media parameters (such as codecs, IP addresses, ports, etc.) between two peers.

## SDP Offer/Answer Model

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