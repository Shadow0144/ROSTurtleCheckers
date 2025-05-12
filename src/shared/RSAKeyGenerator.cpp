#include "shared/RSAKeyGenerator.hpp"

#include <cstdint>
#include <iostream>
#include <random>
#include <cmath>
#include <chrono>
#include <numeric>

#include "shared/CheckersConsts.hpp"

int64_t e = 65537;

void RSAKeyGenerator::generateRSAKeyPair(uint64_t &publicKey, uint64_t &privateKey)
{
    int64_t p;
    int64_t q;
    int64_t n;
    int64_t phi;
    int64_t d;

    do
    {
        do
        {
            p = generateLargePrime(NUM_BITS_RSA);
            do
            {
                q = generateLargePrime(NUM_BITS_RSA);
            } while (p == q); // If they are the same, generate a new q

            n = p * q;
            phi = (p - 1) * (q - 1);
        } while (phi <= e);

        d = modInverse(e, phi);
    } while (d == -1);

    if (d < 0)
    {
        d += phi;
    }

    publicKey = static_cast<uint64_t>(n);
    privateKey = static_cast<uint64_t>(d);
}

// Function to generate a large random prime number
int64_t RSAKeyGenerator::generateLargePrime(uint64_t numBits)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int64_t> distrib(std::pow(2, numBits - 1), std::pow(2, numBits) - 1);

    int64_t largePrime;
    do
    {
        largePrime = distrib(gen);
        largePrime |= 1; // Ensure num is odd
    } while (!isPrime(largePrime));

    return largePrime;
}

// Miller-Rabin primality test
bool RSAKeyGenerator::isPrime(int64_t n, int64_t k)
{
    if (n <= 1)
    {
        return false;
    }
    if (n <= 3)
    {
        return true;
    }
    if (n % 2 == 0)
    {
        return false;
    }

    // Find (s, r) pair for n-1 = 2^s * r such that r is odd
    int64_t s = 0;
    int64_t r = n - 1;
    while (r % 2 == 0)
    {
        s++;
        r /= 2;
    }

    // Do k iterations of Miller-Rabin test
    for (int64_t i = 0; i < k; i++)
    {
        // Generate a random number a in range [2, n-2]
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int64_t> distrib(2, n - 2);
        int64_t a = distrib(gen);

        int64_t x = modularExponentiation(a, r, n);
        if (x == 1 || x == n - 1)
        {
            continue;
        }
        for (int64_t j = 0; j < s - 1; j++)
        {
            x = modularExponentiation(x, 2, n);
            if (x == n - 1)
            {
                break;
            }
        }
        if (x != n - 1)
        {
            return false;
        }
    }
    return true;
}

// Function to calculate modular exponentiation (for Miller-Rabin test)
uint64_t RSAKeyGenerator::modularExponentiation(uint64_t base, uint64_t exp, uint64_t mod)
{
    uint64_t res = 1;
    base %= mod;  // Make sure base is in the correct range [0, mod-1]

    while (exp > 0)
    {
        if (exp % 2 == 1)  // If exp is odd
        {
            res = (res * base) % mod;  // Multiply the result with the base and apply mod
        }
        base = (base * base) % mod;  // Square the base and apply mod
        exp /= 2;  // Halve the exponent
    }

    return res;
}

int64_t RSAKeyGenerator::extendedEuclidean(int64_t a, int64_t b, int64_t &x, int64_t &y)
{
    if (b == 0)
    {
        x = 1;
        y = 0;
        return a;
    }

    int64_t x1;
    int64_t y1;
    int64_t gcd = extendedEuclidean(b, a % b, x1, y1);

    x = y1;
    y = x1 - (a / b) * y1;

    return gcd;
}

int64_t RSAKeyGenerator::modInverse(int64_t e, int64_t phi)
{
    int64_t x;
    int64_t y;
    int64_t gcd = extendedEuclidean(e, phi, x, y);

    if (gcd != 1)
    {
        return -1;
    }

    return (x % phi + phi) % phi;
}

uint64_t RSAKeyGenerator::encrypt(uint64_t message, uint64_t key)
{
    return modularExponentiation(message, e, key);
}

uint64_t RSAKeyGenerator::unencrypt(uint64_t message, uint64_t privateKey, uint64_t publicKey)
{
    return modularExponentiation(message, privateKey, publicKey);
}

uint64_t RSAKeyGenerator::createChecksumSignature(uint64_t hash, uint64_t publicKey, uint64_t privateKey)
{
    uint64_t signature = modularExponentiation(hash, privateKey, publicKey);
    return signature;
}

bool RSAKeyGenerator::checksumSignatureMatches(uint64_t hash, uint64_t publicKey, uint64_t signature)
{
    uint64_t boundHash = hash % publicKey;
    uint64_t recoveredHash = modularExponentiation(signature, e, publicKey);
    return (boundHash == recoveredHash);
}