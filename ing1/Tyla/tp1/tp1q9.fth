\ Syntax to define a function ': <name> returned value ;'
\ Syntax to print '." something" CR'
\ FIXME-begin
: one ." one" CR 1 ;
: two ." two" CR 2 ;
\ FIXME-end
one two two one one -  + - + .
