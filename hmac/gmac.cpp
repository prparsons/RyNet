#include <iostream>

#include <openssl/evp.h>
#include <openssl/err.h>

// def hmacVerifyMessage(signature, bytesSalt, bytesMessage, key):
// def hmacCreateMessage(stringMessage, key):

EVP_PKEY* hmacCreateKey(std::string strKey)
{
    char scKey[EVP_MAX_MD_SIZE];
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
        size = EVP_MD_size(md);
        if(size < 16) {
            printf("EVP_MD_size failed, error 0x%lx\n", ERR_get_error());
            bReturn = false;
        }
    }
    if (bReturn && (size > strKey.length())) {
        printf("EVP_MD_size is too large\n");
        bReturn = false;
    }

    // Create the key itself
    if (bReturn) {
        pKey = EVP_PKEY_new_mac_key(EVP_PKEY_HMAC, NULL, strKey.c_str(), strKey.length());
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

    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pkey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scMessage, uMessageLength) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
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

    // Get the digest and verify size is correct
    size_t* slen = nullptr;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, *scHmac, slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), return code %d, error 0x%lx\n", rc, ERR_get_error());
        bReturn = false;
    }
    if (bReturn && (req != *slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, *slen);
        bReturn = false;
    }

    if (!bReturn) {
        delete scMessage;
        if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx);
    } else {
        EVP_MD_CTX_destroy(pSslCtx);
        *_scMessage = scMessage;
    }

    return bReturn;
}


int hmac_it(const unsigned char *msg, size_t mlen, unsigned char **val, size_t *vlen, EVP_PKEY *pkey)
{
	//* Returned to caller * /
	int result = 0;
	EVP_MD_CTX* ctx = NULL;
	size_t req = 0;
	int rc;

	if(!msg || !mlen || !val || !pkey)
		return 0;

	*val = NULL;
	*vlen = 0;

	ctx = EVP_MD_CTX_new();
	if (ctx == NULL) {
		printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	rc = EVP_DigestSignInit(ctx, NULL, EVP_sha256(), NULL, pkey);
	if (rc != 1) {
		printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	rc = EVP_DigestSignUpdate(ctx, msg, mlen);
	if (rc != 1) {
		printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	rc = EVP_DigestSignFinal(ctx, NULL, &req);
	if (rc != 1) {
		printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
		goto err;
	}

	*val = (unsigned char*)OPENSSL_malloc(req);
	if (*val == NULL) {
		printf("OPENSSL_malloc failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	*vlen = req;
	rc = EVP_DigestSignFinal(ctx, *val, vlen);
	if (rc != 1) {
		printf("EVP_DigestSignFinal failed (3), return code %d, error 0x%lx\n", rc, ERR_get_error());
		goto err;
	}

	result = 1;


err:
	EVP_MD_CTX_free(ctx);
	if (!result) {
		OPENSSL_free(*val);
		*val = NULL;
	}
	return result;
}

int verify_it(const unsigned char *msg, size_t mlen, const unsigned char *val, size_t vlen, EVP_PKEY *pkey)
{
	//* Returned to caller * /
	int result = 0;
	EVP_MD_CTX* ctx = NULL;
	unsigned char buff[EVP_MAX_MD_SIZE];
	size_t size;
	int rc;

	if(!msg || !mlen || !val || !vlen || !pkey)
		return 0;

	ctx = EVP_MD_CTX_new();
	if (ctx == NULL) {
		printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	rc = EVP_DigestSignInit(ctx, NULL, EVP_sha256(), NULL, pkey);
	if (rc != 1) {
		printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	rc = EVP_DigestSignUpdate(ctx, msg, mlen);
	if (rc != 1) {
		printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	size = sizeof(buff);
	rc = EVP_DigestSignFinal(ctx, buff, &size);
	if (rc != 1) {
		printf("EVP_DigestSignFinal failed, error 0x%lx\n", ERR_get_error());
		goto err;
	}

	result = (vlen == size) && (CRYPTO_memcmp(val, buff, size) == 0);
err:
	EVP_MD_CTX_free(ctx);
	return result;
}

int main()
{
    //bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** _scMessage)

    // if(sig)
    //     OPENSSL_free(sig);
    
    // if(skey)
    //     EVP_PKEY_free(skey);

	std::cout << "Hellow world\n";
	return 0;
}

