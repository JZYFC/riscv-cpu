/* Minimal repro: test1 + test2 + test3 from test_add.
 * If this fails at code 3, the bug is in the interaction between
 * test2's execution and test3. Tests 4 and 5 are not needed to trigger it.
 */
__attribute__((noinline)) int add(int a, int b) { return a + b; }
__attribute__((noinline)) int sub(int a, int b) { return a - b; }

int main(void) {
    volatile int a, b, r;

    /* Test 1: add(3, 4) = 7 */
    a = 3; b = 4;
    r = add(a, b);
    if (r != 7) return 1;

    /* Test 2: add(-1, 1) = 0 */
    a = -1; b = 1;
    r = add(a, b);
    if (r != 0) return 2;

    /* Test 3: sub(10, 3) = 7, compare against callee-saved reference (s1=7) */
    a = 10; b = 3;
    r = sub(a, b);
    if (r != 7) return 3;

    return 0;
}
