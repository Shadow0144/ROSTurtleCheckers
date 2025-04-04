#include "RSAKeyGenerator.hpp"

#include <iostream>
#include <random>
#include <cmath>
#include <chrono>
#include <numeric>

#include "CheckersConsts.hpp"

long long e = 65537;

void RSAKeyGenerator::generateRSAKeyPair(long long &publicKey, long long &privateKey)
{
    long long p;
    long long q;
    long long n;
    long long phi;
    long long d;

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

    publicKey = n;
    privateKey = d;
}

// Function to generate a large random prime number
long long RSAKeyGenerator::generateLargePrime(int numBits)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<long long> distrib(std::pow(2, numBits - 1), std::pow(2, numBits) - 1);

    long long largePrime;
    do
    {
        largePrime = distrib(gen);
        largePrime |= 1; // Ensure num is odd
    } while (!isPrime(largePrime));

    return largePrime;
}

// Miller-Rabin primality test
bool RSAKeyGenerator::isPrime(long long n, int k)
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
    long long s = 0;
    long long r = n - 1;
    while (r % 2 == 0)
    {
        s++;
        r /= 2;
    }

    // Do k iterations of Miller-Rabin test
    for (int i = 0; i < k; i++)
    {
        // Generate a random number a in range [2, n-2]
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<long long> distrib(2, n - 2);
        long long a = distrib(gen);

        long long x = modularExponentiation(a, r, n);
        if (x == 1 || x == n - 1)
        {
            continue;
        }
        for (long long j = 0; j < s - 1; j++)
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
long long RSAKeyGenerator::modularExponentiation(long long base, long long exp, long long mod)
{
    long long res = 1;
    base %= mod;
    while (exp > 0)
    {
        if (exp % 2 == 1)
            res = (res * base) % mod;
        base = (base * base) % mod;
        exp /= 2;
    }
    return res;
}

long long RSAKeyGenerator::extendedEuclidean(long long a, long long b, long long &x, long long &y)
{
    if (b == 0)
    {
        x = 1;
        y = 0;
        return a;
    }

    long long x1;
    long long y1;
    long long gcd = extendedEuclidean(b, a % b, x1, y1);

    x = y1;
    y = x1 - (a / b) * y1;

    return gcd;
}

long long RSAKeyGenerator::modInverse(long long e, long long phi)
{
    long long x;
    long long y;
    long long gcd = extendedEuclidean(e, phi, x, y);

    if (gcd != 1)
    {
        return -1;
    }

    return (x % phi + phi) % phi;
}

long long RSAKeyGenerator::createChecksumSignature(long long hash, long long privateKey, long long publicKey)
{
    return (modularExponentiation(hash, privateKey, publicKey));
}

bool RSAKeyGenerator::checksumSignatureMatches(long long hash, long long publicKey, long long signature)
{
    return (modularExponentiation(signature, e, publicKey) == hash);
}