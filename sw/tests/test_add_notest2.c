/* test_add_notest2.c
 * Same as test_add but with test2 (add(-1,1)) removed.
 * If this PASSES but test_add FAILS, then something in test2's
 * execution sequence corrupts state for test3.
 */
__attribute__((noinline)) int add(int a, int b) { return a + b; }
__attribute__((noinline)) int sub(int a, int b) { return a - b; }

int main(void) {
    volatile int a, b, r;

    /* Test 1: 3 + 4 = 7  (stores s1 = 7 in callee-saved reg) */
    a = 3; b = 4;
    r = add(a, b);
    if (r != 7) return 1;

    /* Test 2 REMOVED */

    /* Test 3: sub(10,3) = 7  (compare against s1) */
    a = 10; b = 3;
    r = sub(a, b);
    if (r != 7) return 3;

    return 0;
}
