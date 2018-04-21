changecom(`BLUBLA', `BLABLU')
m(   printB.m4,     0, 0, 0, "" )
divert(-1)
  define(`m4_N', `0')
  define(`m4_next_N', `define(`m4_N', incr(m4_N))')

  define(`m4_print0', `m4_next_N m4_printN(0, $1)')
  define(`m4_print1', `m4_next_N m4_printN(1, $1)')
  define(`m4_print2', `m4_next_N m4_printN(2, $1)')

  define(`m4_printN', `divert(0)m(__file__, __line__, m4_N, $1, $2 )
divert(-1)')

