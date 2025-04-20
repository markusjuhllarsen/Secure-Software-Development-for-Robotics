import secrets
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.hashes import SHA256

def encrypt_and_gmac(aesgcm, message, timestamp):
    """
    Encrypt a message and generate GMAC to ensure confidentiality and integrity
    Args:
        message (str): The command message to generate the MAC for.

    Returns:
        str: The GMAC as a hexadecimal string.
    """
    data = f"{message}:{timestamp}".encode()
    nonce = secrets.token_bytes(12)
    encrypted_data = aesgcm.encrypt(nonce, data, None)

    return nonce, encrypted_data
    
def decrypt_and_verify(aesgcm, encrypted_data, nonce):
    """
    Decrypt the message and verify GMAC to ensure confidentiality and integrity
    Args:
        encrypted_data (bytes): The encrypted command message.
        nonce (bytes): The nonce used for encryption.

    Returns:
        str: The decrypted message.
    """
    try:
        # Decrypt the data and verify GMAC
        # The GMAC is verified during decryption, so if it fails, an exception will be raised
        decrypted_data = aesgcm.decrypt(nonce, encrypted_data, None).decode()
        message, _ = decrypted_data.split(':', 1)  # Split by the last colon
        return message
    except Exception as e:
        return None
    
def exchange_keys(private_key, peer_public_key_bytes):
    """
    Perform ECDH key exchange to derive a shared AES key.
    Args:
        private_key (PrivateKey): The private key of the local node.
        peer_public_key_bytes (bytes): The public key of the peer in PEM format.
    """
    try:
        # Deserialize the peer's public key
        peer_public_key = serialization.load_pem_public_key(peer_public_key_bytes)

        # Compute the shared secret
        shared_secret = private_key.exchange(ec.ECDH(), peer_public_key)

        # Derive the AES key using HKDF
        aes_key = HKDF(
            algorithm=SHA256(),
            length=32,  # AES-256 key length
            salt=None,
            info=b"handshake data"
        ).derive(shared_secret)

        return aes_key
    except Exception as e:
        return None