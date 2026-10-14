// assert_recover.cocci -- rules 5 and 7 (docs/CODING_STANDARD.md): an
// assertion must have an explicit recovery; `(void) AGC_ASSERT(cond);` has
// none. The mechanical cases, keyed on the enclosing function's return type:
//
//   spatch --sp-file tools/archive/assert_recover.cocci --in-place --dir src \
//          -I include
//
//   agc_status_t function: if (!AGC_ASSERT(c)) { return AGC_STATUS_BAD_ARG; }
//   void function:         if (!AGC_ASSERT(c)) { return; }
//   bool function:         if (!AGC_ASSERT(c)) { return false; }
//
// Value-returning numeric helpers need a documented safe value (0 or the
// saturated bound the AGC would hold), constant conditions become
// _Static_assert, `AGC_ASSERT(false)` markers keep only their recovery, an
// assertion inside INHINT also needs agc_relint() before the return, and
// dead checks in pointer accessors are deleted: all done by hand (see
// docs/history/waves/status-handling.md).

@status@
typedef agc_status_t;
identifier fn;
expression E;
@@
agc_status_t fn(...)
{
<...
- (void) AGC_ASSERT(E);
+ if (!AGC_ASSERT(E))
+ {
+     return AGC_STATUS_BAD_ARG;
+ }
...>
}

@voidfn@
identifier fn;
expression E;
@@
void fn(...)
{
<...
- (void) AGC_ASSERT(E);
+ if (!AGC_ASSERT(E))
+ {
+     return;
+ }
...>
}

@boolfn@
typedef bool;
identifier fn;
expression E;
@@
bool fn(...)
{
<...
- (void) AGC_ASSERT(E);
+ if (!AGC_ASSERT(E))
+ {
+     return false;
+ }
...>
}
