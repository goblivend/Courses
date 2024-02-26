package main

import (
	"fmt"
)

type Duck interface {
   Quacks()
}

func Quack(duck Duck) {
   duck.Quacks()
}

// Define a RealDuck structure that can acts as a Duck
// FIXME-begin
type RealDuck struct {
   Duck
}

func (a RealDuck) Quacks() {
   fmt.Println("RealDuck Quacks");
}
// FIXME-end



type Unicorn struct {
    name string
}

func (a Unicorn) DoMagic() {
   fmt.Println("Unicorn do Magic");
}


// Extends the Unicorn structure that can acts as a Duck
// FIXME-begin
func (a Unicorn) Quacks() {
   fmt.Println("Unicorn Quacks");
}
// FIXME-end

func main() {
   d := RealDuck{}
   Quack(d)
   a := Unicorn{}
   a.DoMagic()
   Quack(a)
}
