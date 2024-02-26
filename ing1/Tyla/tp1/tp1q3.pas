//fpc 3.0.0
program TP1Q3;

// FIXME-begin
Procedure one (tw: String; th: String);
begin
       writeln('one');
end;

Function two () : String;
begin
       writeln('two');
       two := '';
end;

Function three () : String;
begin
       writeln('three');
       three := '';
end;
// FIXME-end

begin
    one(two(), three());
    writeln('Pascal!');
end.
