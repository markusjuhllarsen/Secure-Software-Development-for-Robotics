import secrets

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