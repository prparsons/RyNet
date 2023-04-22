#include <iostream>

#include <openssl/evp.h>
#include <openssl/err.h>

int hmac_it(const unsigned char *msg, size_t mlen, unsigned char **val, size_t *vlen, EVP_PKEY *pkey)
{
	/* Returned to caller */
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
	/* Returned to caller */
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
	std::cout << "Hellow world\n";
	return 0;
}
