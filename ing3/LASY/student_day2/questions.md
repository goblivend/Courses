# Contracts OOP (Ada 2012)

The purpose of this exercise is to understand contracts in the OOP context.

*Note : Follow the instructions, keep the structure of the folders, do not rename files, use already provided functions and macros, do not rename provided functions and macros *

## Part 1 (Go to part1 folder)
### Question 1 
Write a simple contract for the primitive Squared_Norm of Coordinates stating that its
result shall always be greater or equal to zero.
Testing with the main subprogram, what can you say about the primitive Squared_Norm
of the type Vector derived from the type Coordinates?

Provide your answer in the answer.txt file.

## Part 2 (Go to part2 folder)
### Question 1
Ordered_Structure is an abstract class representing container structures whose elements are in
ascending order.

Write type invariant for this class specifying this property.

*Note : Only edit specification files (*.ads)*

### Question 2
In order to delete an element, from an Ordered_Structure, it should be contained in the structure.

Write a contract stating this property.

*Note : Only edit specification files (*.ads)*

### Question 3
The primitive (method) Insert adds an element to the Ordered_Structure, the primitive
Delete removes an element to the Ordered_Structure.
Write postconditions describing the functional properties of these primitives.

[hint: create an auxiliary function to write the contract]

[Note: no need to check that the set is ordered in the postcondition or that the element is
added at the right position, this is already automatically checked with our type
invariant]

*Note : Only edit specification files (*.ads)*

### Question 4
A set is a container with no duplicated element. Could we create a class Ordered_Set
inheriting from Ordered_Structure?

Provide your answer in the answer.txt file


# Submission
Once you have done the exercises keeping the folder structure provided
create a zip file in place named student_day2.zip containing the same folder structure and files and submit the zip file on Moodle.

