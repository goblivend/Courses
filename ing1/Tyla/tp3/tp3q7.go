package main

import (
	"fmt"
)

type MyFIFO []interface{}

// Define a Push mthod for MyFIFO
// FIXME-begin
func (f *MyFIFO) Push(elem interface{}) {
	*f = append(*f, elem)
}
// FIXME-end

// Define a Pop mthod for MyFIFO
// FIXME-begin
func (f *MyFIFO) Pop() interface{} {
	elem := (*f)[0]
	*f = (*f)[1:]
	return elem
}
// FIXME-end

func main() {

	// Build an int FIFO Container
	intContainer := &MyFIFO{}
	intContainer.Push(42)
	intContainer.Push(51)
	intContainer.Push(69)
	intContainer.Push(1337)
	for len(*intContainer) > 0 {
		fmt.Println("size:", len(*intContainer))
		elem, _ := intContainer.Pop().(int) // assert that the actual type is int
		fmt.Printf("element %d with type %T\n", elem, elem)
	}

	// Build a string FIFO Container
	stringContainer := &MyFIFO{}
	stringContainer.Push("aa")
	stringContainer.Push("bb")
	stringContainer.Push("cc")
	stringContainer.Push("dd")
	fmt.Println("---")
	for len(*stringContainer) > 0 {
		fmt.Println("size:", len(*stringContainer))
		elem, _ := stringContainer.Pop().(string) // assert that the actual type is string
		fmt.Printf("element %s with type %T\n", elem, elem)
	}

	// Build a mixed string/int FIFO Container
	mixedContainer := &MyFIFO{}
	mixedContainer.Push("aa")
	mixedContainer.Push(42)
	fmt.Println("---")
	for len(*mixedContainer) > 0 {
		fmt.Println("size:", len(*mixedContainer))
		fmt.Println(mixedContainer.Pop())
	}
}
