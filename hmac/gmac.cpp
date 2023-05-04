#include <iostream>

#include <string.h>
#include <openssl/evp.h>
#include <openssl/err.h>

// def hmacVerifyMessage(signature, bytesSalt, bytesMessage, key):
// def hmacCreateMessage(stringMessage, key):

EVP_PKEY* hmacCreateKey(std::string strKey)
{
    char scKey[EVP_MAX_MD_SIZE];
    for (int i = 0; i < EVP_MAX_MD_SIZE; i++) {
        scKey[i] = 0;
    }
    strncpy(scKey, strKey.c_str(), strKey.length());
    //**not padding the key with anything
    EVP_PKEY* pKey = nullptr;

    // Get the key type, create the digest
    bool bReturn = true;
    const EVP_MD* pMd = EVP_get_digestbyname("SHA256");
    if (pMd == nullptr) {
        printf("EVP_get_digestbyname failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    int size = 0;
    if (bReturn) {
        size = EVP_MD_size(pMd);
        if(size < 16) {
            printf("EVP_MD_size failed, error 0x%lx\n", ERR_get_error());
            bReturn = false;
        } else {
            printf("Key size%d, md_size%d\n", int(strKey.length()), size);
        }
    }
    //if (bReturn && (size > strKey.length())) {
    //    printf("EVP_MD_size is too large\n");
    //    bReturn = false;
    //}

    // Create the key itself
    if (bReturn) {
        pKey = EVP_PKEY_new_mac_key(EVP_PKEY_HMAC, NULL, (const unsigned char*)scKey, size);
        if (pKey == nullptr) {
            printf("EVP_PKEY_new_mac_key failed, error 0x%lx\n", ERR_get_error());
            bReturn = false;
        }
    }
    if (!bReturn && pKey != nullptr) {
        printf("Should be key failure, but it's not\n");
    }
    
    return pKey;
}

bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** _scMessage)
{
    *scHmac = nullptr;
    *_scMessage = nullptr;

    size_t uMessageLength = strMessage.length() + 4;
    char* scMessage = new char[uMessageLength];
    *((int32_t*)scMessage) = iSalt;
    strncpy(scMessage + 4, strMessage.c_str(), strMessage.length());
    bool bReturn = true;

    // Create the message digest context
    EVP_MD_CTX* pSslCtx = nullptr;
    pSslCtx = EVP_MD_CTX_create();
    if (pSslCtx == nullptr) {
        printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
        delete[] scMessage;
        return false;
    }
    // Get the message digest for sha256
    const EVP_MD* pMd = EVP_get_digestbyname("SHA256");
    if(pMd == nullptr) {
        printf("EVP_get_digestbyname failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    // Create the hmac/sha256 digest
    if (bReturn && (EVP_DigestInit_ex(pSslCtx, pMd, NULL) != 1)) {
        printf("EVP_DigestInit_ex failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    EVP_PKEY* pKey = nullptr;
    if (bReturn) {
        pKey = hmacCreateKey(strKey);
        if (pKey == nullptr) bReturn = false;
    }
    printf("key creation done\n");
    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pKey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign init done\n");
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scMessage, uMessageLength) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign update done\n");

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final done\n");
    if (bReturn && (req == 0)) {
        printf("EVP_DigestSignFinal failed (2), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    } else {
        *scHmac = (char*)OPENSSL_malloc(req);
    }
    if (bReturn && (scHmac == nullptr)) {
        printf("OPENSSL_malloc failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("ssl malloc done\n");

    // Get the digest and verify size is correct
    size_t slen = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, (unsigned char*)(*scHmac), &slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final 2 done\n");
    if (bReturn && (req != slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, slen);
        bReturn = false;
    }

    if (!bReturn) {
        printf("Failed to hmac things\n");
        delete scMessage;
        if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx);
    } else {
        printf("hmac successful, cleaning up\n");
        EVP_MD_CTX_destroy(pSslCtx);
        *_scMessage = scMessage;
    }

    return bReturn;
}

bool hmacBuffer(std::string strMessage, std::string strKey, char** scHmac)
{
    *scHmac = nullptr;

    size_t uMessageLength = strMessage.length();
    char* scMessage = new char[uMessageLength];
    strncpy(scMessage, strMessage.c_str(), strMessage.length());
    bool bReturn = true;

    // Create the message digest context
    EVP_MD_CTX* pSslCtx = nullptr;
    pSslCtx = EVP_MD_CTX_create();
    if (pSslCtx == nullptr) {
        printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
        delete[] scMessage;
        return false;
    }
    // Get the message digest for sha256
    const EVP_MD* pMd = EVP_get_digestbyname("SHA256");
    if(pMd == nullptr) {
        printf("EVP_get_digestbyname failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    // Create the hmac/sha256 digest
    if (bReturn && (EVP_DigestInit_ex(pSslCtx, pMd, NULL) != 1)) {
        printf("EVP_DigestInit_ex failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    EVP_PKEY* pKey = nullptr;
    if (bReturn) {
        pKey = hmacCreateKey(strKey);
        if (pKey == nullptr) bReturn = false;
    }
    printf("key creation done\n");
    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pKey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign init done\n");
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scMessage, uMessageLength) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign update done\n");

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final done\n");
    if (bReturn && (req == 0)) {
        printf("EVP_DigestSignFinal failed (2), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    } else {
        *scHmac = (char*)OPENSSL_malloc(req);
    }
    if (bReturn && (scHmac == nullptr)) {
        printf("OPENSSL_malloc failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("ssl malloc done\n");

    // Get the digest and verify size is correct
    size_t slen = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, (unsigned char*)(*scHmac), &slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final 2 done\n");
    if (bReturn && (req != slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, slen);
        bReturn = false;
    }

    if (!bReturn) {
        printf("Failed to hmac things\n");
        delete scMessage;
        if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx);
    } else {
        printf("hmac successful, cleaning up\n");
        EVP_MD_CTX_destroy(pSslCtx);
    }

    return bReturn;
}

int main()
{
    //bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** _scMessage)

    // if(sig)
    //     OPENSSL_free(sig);
    
    // if(skey)
    //     EVP_PKEY_free(skey);
    //EVP_PKEY* key = hmacCreateKey("Secret Key");
    char* scHmac;
    char* scMessage;
    //if (!hmacMessage("<pos_retrieve />", "Secret Key", 0, &scHmac, &scMessage)) {
    if (!hmacBuffer("<pos_retrieve />", "Secret Key", &scHmac)) {
        printf("Nope\n");
    }
    for (int i = 0; i < 32; i++) {
        printf("%02X", ((unsigned char*)scHmac)[i]);
    }
    std::cout << "\nHello world\n";
    return 0;
}

