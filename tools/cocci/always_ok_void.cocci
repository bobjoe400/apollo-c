// always_ok_void.cocci -- rule 7 (docs/CODING_STANDARD.md): the functions
// tools/always_ok.py reports (every return is AGC_STATUS_OK, nothing can
// fail) become void, and their callers' status plumbing goes.  The regex
// below is the record of one wave (docs/modules/always-ok.md); the file is
// regenerated per wave.
//
//   spatch --sp-file tools/cocci/always_ok_void.cocci --in-place \
//          -I include --include-headers <files>
//
// Shapes the rules do not cover (status chains, ternaries, a status kept
// for a later test) are fixed with the compiler's help: "void value not
// ignored", then the dead tests of the status variable go.

// 1. The definition and its prototypes: agc_status_t -> void.
@def@
typedef agc_status_t;
identifier f =~ "^(laim_apsides)$";
parameter list ps;
@@
- agc_status_t
+ void
  f(ps)
  {
  ...
  }

@proto@
typedef agc_status_t;
identifier f =~ "^(laim_apsides)$";
parameter list ps;
@@
- agc_status_t
+ void
  f(ps);

// 2. Its returns become `return;`.  The final one is then deleted by
//    tools/drop_final_return.py (a path-based "... return; }" rule would
//    also delete early returns: every return reaches the end of the
//    function).
@ret@
identifier f =~ "^(laim_apsides)$";
@@
void f(...)
{
<...
(
- return (AGC_STATUS_OK);
+ return;
|
- return AGC_STATUS_OK;
+ return;
)
...>
}

// 3. Callers.
@voidcast@
identifier f =~ "^(laim_apsides)$";
expression list es;
@@
- (void) f(es);
+ f(es);

@retvoid@
identifier fn;
identifier f =~ "^(laim_apsides)$";
expression list es;
@@
void fn(...)
{
<...
(
- return (f(es));
+ f(es);
+ return;
|
- return f(es);
+ f(es);
+ return;
)
...>
}

@retok@
identifier f =~ "^(laim_apsides)$";
expression list es;
@@
(
- return (f(es));
+ f(es);
+ return (AGC_STATUS_OK);
|
- return f(es);
+ f(es);
+ return (AGC_STATUS_OK);
)

@checked@
identifier st;
identifier f =~ "^(laim_apsides)$";
expression list es;
type T;
@@
(
- st = f(es);
+ f(es);
|
- T st = f(es);
+ f(es);
)
- if (st != AGC_STATUS_OK)
- {
-     return (st);
- }

@test1@
identifier f =~ "^(laim_apsides)$";
expression list es;
@@
- CHECK(f(es) == AGC_STATUS_OK);
+ f(es);

@test2@
identifier f =~ "^(laim_apsides)$";
expression list es;
expression E;
@@
- CHECK(f(es) == AGC_STATUS_OK && E);
+ f(es);
+ CHECK(E);
