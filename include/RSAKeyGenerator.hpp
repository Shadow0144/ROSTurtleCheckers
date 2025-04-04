#pragma once

class RSAKeyGenerator
{
public:
    void generateRSAKeyPair(long long &publicKey, long long &privateKey);
    
    long long createChecksumSignature(long long hash, long long privateKey, long long publicKey);
    bool checksumSignatureMatches(long long hash, long long publicKey, long long signature);

private:
    long long generateLargePrime(int numBits);
    bool isPrime(long long n, int k = 5);
    long long modularExponentiation(long long base, long long exp, long long mod);
    long long extendedEuclidean(long long a, long long b, long long &x, long long &y);
    long long modInverse(long long e, long long phi);
};