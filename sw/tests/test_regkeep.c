/* test_regkeep.c — Reproduces the pattern that fails in test_add:
 *   1. Compute a value into a callee-saved register (s1 = 7)
 *   2. Do a second function call (possible branch misprediction recovery)
 *   3. Check that s1 still holds 7 when used as reference for the THIRD call
 *
 * Mirrors exactly the structure of test_add's tests 1→2→3.
 */
__attribute__((noinline)) int sub_fn(int a, int b) { return a - b; }
__attribute__((noinline)) int add_fn(int a, int b) { return a + b; }

int main(void) {
    volatile int a, b, r;

    /* === PHASE 1: add(3,4) → result stored in callee-saved reg (s1) === */
    a = 3; b = 4;
    r = add_fn(a, b);               /* r = 7 */
    if (r != 7) return 1;
    /* Compiler keeps r=7 in s1 across subsequent calls */

    /* === PHASE 2: add(-1,1) → independent result, similar to test_add test 2 === */
    a = -1; b = 1;
    volatile int r2 = add_fn(a, b); /* r2 = 0 */
    if (r2 != 0) return 2;

    /* === PHASE 3: sub(10,3) compared against PHASE 1's result (s1=7) === */
    /* This is the exact pattern that fails in test_add */
    a = 10; b = 3;
    volatile int r3 = sub_fn(a, b); /* r3 = 7 */
    if (r3 != r) return 3;          /* compare against s1 (from phase 1) */

    return 0;
}
