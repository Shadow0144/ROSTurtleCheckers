#pragma once

#include <cstdint>

class RSAKeyGenerator
{
public:
    static void generateRSAKeyPair(uint64_t &publicKey, uint64_t &privateKey);
    
    static uint64_t encrypt(uint64_t message, uint64_t key);
    static uint64_t unencrypt(uint64_t message, uint64_t privateKey, uint64_t publicKey);

    static uint64_t createChecksumSignature(uint64_t hash, uint64_t publicKey, uint64_t privateKey);
    static bool checksumSignatureMatches(uint64_t hash, uint64_t publicKey, uint64_t signature);

private:
    static int64_t generateLargePrime(uint64_t numBits);
    static bool isPrime(int64_t n, int64_t k = 5);
    static uint64_t modularExponentiation(uint64_t base, uint64_t exp, uint64_t mod);
    static int64_t extendedEuclidean(int64_t a, int64_t b, int64_t &x, int64_t &y);
    static int64_t modInverse(int64_t e, int64_t phi);
};