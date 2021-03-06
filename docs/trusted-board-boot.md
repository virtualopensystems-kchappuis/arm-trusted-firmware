Trusted Board Boot Design Guide
===============================

Contents :

1.  [Introduction](#1--introduction)
2.  [Chain of Trust](#2--chain-of-trust)
3.  [Trusted Board Boot Sequence](#3--trusted-board-boot-sequence)
4.  [Authentication Framework](#4--authentication-framework)
5.  [Certificate Generation Tool](#5--certificate-generation-tool)


1.  Introduction
----------------

The Trusted Board Boot (TBB) feature prevents malicious firmware from running on
the platform by authenticating all firmware images up to and including the
normal world bootloader. It does this by establishing a Chain of Trust using
Public-Key-Cryptography Standards (PKCS).

This document describes the design of the ARM Trusted Firmware TBB
implementation. The current implementation is a proof of concept; future
versions will provide stronger architectural interfaces and implement the
missing functionality required in a production TBB-enabled system.


2.  Chain of Trust
------------------

A Chain of Trust (CoT) starts with a set of implicitly trusted components. On
the ARM development platforms, these components are:

*   A SHA-256 hash of the Root of Trust Public Key (ROTPK). It is stored in the
    trusted root-key storage registers.

*   The BL1 image, on the assumption that it resides in ROM so cannot be
    tampered with.

The remaining components in the CoT are either certificates or boot loader
images. The certificates follow the [X.509 v3] standard. This standard
enables adding custom extensions to the certificates, which are used to store
essential information to establish the CoT.

In the TBB CoT all certificates are self-signed. There is no need for a
Certificate Authority (CA) because the CoT is not established by verifying the
validity of a certificate's issuer but by the content of the certificate
extensions. To sign the certificates, the PKCS#1 SHA-256 with RSA Encryption
signature scheme is used with a RSA key length of 2048 bits. Future version of
Trusted Firmware will support additional cryptographic algorithms.

The certificates are categorised as "Key" and "Content" certificates. Key
certificates are used to verify public keys which have been used to sign content
certificates. Content certificates are used to store the hash of a boot loader
image. An image can be authenticated by calculating its hash and matching it
with the hash extracted from the content certificate. The SHA-256 function is
used to calculate all hashes. The public keys and hashes are included as
non-standard extension fields in the [X.509 v3] certificates.

The keys used to establish the CoT are:

*   **Root of trust key**

    The private part of this key is used to sign the BL2 content certificate and
    the trusted key certificate. The public part is the ROTPK.

*   **Trusted world key**

    The private part is used to sign the key certificates corresponding to the
    secure world images (BL3-0, BL3-1 and BL3-2). The public part is stored in
    one of the extension fields in the trusted world certificate.

*   **Non-trusted world key**

    The private part is used to sign the key certificate corresponding to the
    non secure world image (BL3-3). The public part is stored in one of the
    extension fields in the trusted world certificate.

*   **BL3-X keys**

    For each of BL3-0, BL3-1, BL3-2 and BL3-3, the private part is used to sign
    the content certificate for the BL3-X image. The public part is stored in
    one of the extension fields in the corresponding key certificate.

The following images are included in the CoT:

*   BL1
*   BL2
*   BL3-0 (optional)
*   BL3-1
*   BL3-3
*   BL3-2 (optional)

The following certificates are used to authenticate the images.

*   **BL2 content certificate**

    It is self-signed with the private part of the ROT key. It contains a hash
    of the BL2 image.

*   **Trusted key certificate**

    It is self-signed with the private part of the ROT key. It contains the
    public part of the trusted world key and the public part of the non-trusted
    world key.

*   **BL3-0 key certificate**

    It is self-signed with the trusted world key. It contains the public part of
    the BL3-0 key.

*   **BL3-0 content certificate**

    It is self-signed with the BL3-0 key. It contains a hash of the BL3-0 image.

*   **BL3-1 key certificate**

    It is self-signed with the trusted world key. It contains the public part of
    the BL3-1 key.

*   **BL3-1 content certificate**

    It is self-signed with the BL3-1 key. It contains a hash of the BL3-1 image.

*   **BL3-2 key certificate**

    It is self-signed with the trusted world key. It contains the public part of
    the BL3-2 key.

*   **BL3-2 content certificate**

    It is self-signed with the BL3-2 key. It contains a hash of the BL3-2 image.

*   **BL3-3 key certificate**

    It is self-signed with the non-trusted world key. It contains the public
    part of the BL3-3 key.

*   **BL3-3 content certificate**

    It is self-signed with the BL3-3 key. It contains a hash of the BL3-3 image.

The BL3-0 and BL3-2 certificates are optional, but they must be present if the
corresponding BL3-0 or BL3-2 images are present.


3.  Trusted Board Boot Sequence
-------------------------------

The CoT is verified through the following sequence of steps. The system panics
if any of the steps fail.

*   BL1 loads and verifies the BL2 content certificate. The issuer public key is
    read from the verified certificate. A hash of that key is calculated and
    compared with the hash of the ROTPK read from the trusted root-key storage
    registers. If they match, the BL2 hash is read from the certificate.

    Note: the matching operation is platform specific and is currently
    unimplemented on the ARM development platforms.

*   BL1 loads the BL2 image. Its hash is calculated and compared with the hash
    read from the certificate. Control is transferred to the BL2 image if all
    the comparisons succeed.

*   BL2 loads and verifies the trusted key certificate. The issuer public key is
    read from the verified certificate. A hash of that key is calculated and
    compared with the hash of the ROTPK read from the trusted root-key storage
    registers. If the comparison succeeds, BL2 reads and saves the trusted and
    non-trusted world public keys from the verified certificate.

The next two steps are executed for each of the BL3-0, BL3-1 & BL3-2 images. The
steps for the optional BL3-0 and BL3-2 images are skipped if these images are
not present.

*   BL2 loads and verifies the BL3-x key certificate. The certificate signature
    is verified using the trusted world public key. If the signature
    verification succeeds, BL2 reads and saves the BL3-x public key from the
    certificate.

*   BL2 loads and verifies the BL3-x content certificate. The signature is
    verified using the BL3-x public key. If the signature verification succeeds,
    BL2 reads and saves the BL3-x image hash from the certificate.

The next two steps are executed only for the BL3-3 image.

*   BL2 loads and verifies the BL3-3 key certificate. If the signature
    verification succeeds, BL2 reads and saves the BL3-3 public key from the
    certificate.

*   BL2 loads and verifies the BL3-3 content certificate. If the signature
    verification succeeds, BL2 reads and saves the BL3-3 image hash from the
    certificate.

The next step is executed for all the boot loader images.

*   BL2 calculates the hash of each image. It compares it with the hash obtained
    from the corresponding content certificate. The image authentication succeeds
    if the hashes match.

The Trusted Board Boot implementation spans both generic and platform-specific
BL1 and BL2 code, and in tool code on the host build machine. The feature is
enabled through use of specific build flags as described in the [User Guide].

On the host machine, a tool generates the certificates, which are included in
the FIP along with the boot loader images. These certificates are loaded in
Trusted SRAM using the IO storage framework. They are then verified by an
Authentication module included in the Trusted Firmware.

The mechanism used for generating the FIP and the Authentication module are
described in the following sections.


4.  Authentication Framework
----------------------------

The authentication framework included in the Trusted Firmware provides support
to implement the desired trusted boot sequence. ARM platforms use this framework
to implement the boot requirements specified in the TBBR-client document.

More information about the authentication framework can be found in the
[Auth Framework] document.


5.  Certificate Generation Tool
-------------------------------

The `cert_create` tool is built and runs on the host machine as part of the
Trusted Firmware build process when `GENERATE_COT=1`. It takes the boot loader
images and keys as inputs (keys must be in PEM format) and generates the
certificates (in DER format) required to establish the CoT. New keys can be
generated by the tool in case they are not provided. The certificates are then
passed as inputs to the `fip_create` tool for creating the FIP.

The certificates are also stored individually in the in the output build
directory.

The tool resides in the `tools/cert_create` directory. It uses OpenSSL SSL
library version 1.0.1 or later to generate the X.509 certificates. Instructions
for building and using the tool can be found in the [User Guide].


- - - - - - - - - - - - - - - - - - - - - - - - - -

_Copyright (c) 2015, ARM Limited and Contributors. All rights reserved._


[X.509 v3]:          http://www.ietf.org/rfc/rfc5280.txt
[X.690]:             http://www.itu.int/ITU-T/studygroups/com17/languages/X.690-0207.pdf
[Auth Framework]:    auth-framework.md
[User Guide]:        user-guide.md
