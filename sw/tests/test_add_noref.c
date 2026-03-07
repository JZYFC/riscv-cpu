/* test_add_noref.c
 *
 * Same as test_add.c but test1 produces 8 (add(4,4)) instead of 7.
 * This forces the compiler to NOT reuse s1 (which holds 8) for test3's
 * comparison against 7, so test3 gets: lw a5,r; bne a5,7 (literal),
 * instead of: lw a5,r; bne a5,s1.
 *
 * Diagnostic:
 *   If test_add FAILS code 3 but this PASSES  → s1 is corrupted (register rename bug)
 *   If this also FAILS code 3                 → sub result or store/load is wrong
 */
__attribute__((noinline)) int add(int a, int b) { return a + b; }
__attribute__((noinline)) int sub(int a, int b) { return a - b; }

int main(void) {
    volatile int a, b, r;

    /* Test 1: 4 + 4 = 8  (NOT 7, so compiler keeps s1=8 which != 7) */
    a = 4; b = 4;
    r = add(a, b);
    if (r != 8) return 1;

    /* Test 2: -1 + 1 = 0 */
    a = -1; b = 1;
    r = add(a, b);
    if (r != 0) return 2;

    /* Test 3: sub(10,3) = 7, compared against literal 7 (NOT s1=8) */
    a = 10; b = 3;
    r = sub(a, b);
    if (r != 7) return 3;

    /* Tests 4+5: identical to test_add.c to preserve register allocation */
    volatile int sum = 0;
    volatile int i;
    for (i = 1; i <= 10; i++)
        sum = add(sum, i);
    if (sum != 55) return 4;

    r = add(sub(8, 3), add(2, 2));
    if (r != 9) return 5;

    return 0;
}
