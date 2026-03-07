/* test_regkeep2.c
 *
 * Forces compiler to keep 'expected' in a callee-saved register across
 * multiple function calls — exactly the pattern test_add uses for s1.
 *
 * Non-volatile 'expected' means the compiler will hold it in s0/s1 rather
 * than reloading from memory for each use.
 */
__attribute__((noinline)) int sub_fn(int a, int b) { return a - b; }
__attribute__((noinline)) int add_fn(int a, int b) { return a + b; }

int main(void) {
    volatile int a, b;

    /* Phase 1: compute 'expected' — compiler keeps this in a callee-saved reg */
    a = 3; b = 4;
    int expected = add_fn(a, b);   /* NOT volatile: kept in s0/s1 by compiler */
    if (expected != 7) return 1;   /* branch whose WRONG PATH might rename that reg */

    /* Phase 2: another call (similar to test_add test 2) */
    a = -1; b = 1;
    int r2 = add_fn(a, b);
    if (r2 != 0) return 2;

    /* Phase 3: sub result compared against 'expected' still in register */
    a = 10; b = 3;
    int r3 = sub_fn(a, b);
    if (r3 != expected) return 3;  /* KEY: expected read from register, not memory */

    return 0;
}
