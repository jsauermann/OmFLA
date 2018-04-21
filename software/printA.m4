changecom(`BLUBLA', `BLABLU')
divert(-1)
  define(`m4_N', `0')
  define(`m4_next_N', `define(`m4_N', incr(m4_N))')

  define(`m4_print0', `m4_next_N print0(m4_N)')
  define(`m4_print1', `m4_next_N print1(m4_N, $2)')
  define(`m4_print2', `m4_next_N print2(m4_N, $2, $3)')

divert(0)

