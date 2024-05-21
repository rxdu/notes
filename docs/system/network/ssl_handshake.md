# SSL/TLS Handshake Process

!!! info 

    This note was written with the assistance of ChatGPT.

The following is an overview of the SSL/TLS handshake process:

### 1. Verification of Trusted Entities Using CA Certificates

- **Purpose**: The primary purpose of the CA certificate is to establish trust between the communicating entities (server and client).
- **How it works**: When a client connects to a server, the server presents its certificate to the client. This certificate includes the server's public key and is signed by a trusted Certificate Authority (CA). The client uses the CA's certificate (which includes the CA's public key) to verify the server's certificate. If the verification is successful, it means the server's certificate is trusted.
- **Mutual TLS**: In mutual TLS (mTLS), the client also presents its certificate to the server, and the server verifies it using the CA's certificate.

### 2. Exchange of Public Keys

- **Initial Handshake**: During the SSL/TLS handshake, the server and client exchange their public keys. This is part of the certificate exchange process.
- **Server Key Exchange**: The server sends its public key to the client in its certificate. If using mTLS, the client also sends its public key to the server in its certificate.
- **Purpose**: These public keys are used to securely exchange a session key, which will be used for encrypting the data during the session.

### 3. Encryption and Decryption Using Public and Private Keys

- **Public Key Encryption**: Once the public keys are exchanged, the client and server can encrypt data using the public key of the other party. However, in most SSL/TLS implementations, this direct encryption/decryption with public/private keys is used primarily for secure key exchange rather than for the bulk data encryption.
- **Session Key**: The actual data encryption uses symmetric encryption with a session key (also called a shared secret). The session key is exchanged securely using the public keys of the server and client.
  - **RSA Key Exchange**: The client generates a session key, encrypts it with the server’s public key, and sends it to the server. Only the server, with its private key, can decrypt this session key.
  - **Diffie-Hellman Key Exchange**: The client and server use the Diffie-Hellman algorithm (or its elliptic curve variant) to securely agree on a session key over an insecure channel without directly exchanging the key.

- **Data Encryption**: After the session key is established:
  - The client and server use this symmetric session key to encrypt and decrypt the data they send to each other.
  - Symmetric encryption (e.g., AES) is used for the actual data encryption because it is much faster than asymmetric encryption.

### Detailed Steps in an SSL/TLS Handshake

1. **Client Hello**:
   - The client sends a "Client Hello" message to the server, which includes supported SSL/TLS versions, cipher suites, and a random number.

2. **Server Hello**:
   - The server responds with a "Server Hello" message, including the chosen SSL/TLS version, cipher suite, and a random number.

3. **Server Certificate**:
   - The server sends its certificate to the client. This certificate contains the server’s public key and is signed by a CA.

4. **(Optional) Client Certificate**:
   - In mTLS, the server also requests the client’s certificate, and the client sends it in this step.

5. **Key Exchange**:
   - **RSA**: The client generates a session key, encrypts it with the server’s public key, and sends it to the server.
   - **Diffie-Hellman**: The client and server exchange Diffie-Hellman parameters to securely agree on a session key.

6. **Server and Client Finished**:
   - Both the server and client send a "Finished" message encrypted with the session key, indicating that the handshake is complete.

7. **Secure Communication**:
   - All subsequent data is encrypted with the symmetric session key established during the handshake.

### Summary

- **CA Certificate**: Used to verify the authenticity of the server and client certificates, ensuring they are trusted entities.
- **Public Key Exchange**: Public keys are exchanged to securely establish a session key.
- **Session Key**: A symmetric session key is used for encrypting and decrypting the actual data during the communication.

By following this process, SSL/TLS ensures secure and authenticated communication between the client and server.