# Contracts proof (SPARK)

The purpose of these exercises is to statically prove the absence of runtime errors, contracts and properties of the source code.

*Note : Follow the instructions, keep the structure of the folders, do not rename files, use already provided functions and macros, do not rename provided functions and macros *

*Running gnatprove trough Alire*
```console
alr gnatprove -P myproject.gpr -j0 --level=0 --output=oneline  --report=all
```
*Setting up your environment :*
```console
eval "$(alr printenv)"
gnatprove -P myproject.gpr -j0 --level=0 --output=oneline  --report=all
```

## Part 1 - Dynmaic to static analysis (Go to stack folder)
### Question 1
Build and prove the stack package.

## Part 2 - Proving Loops Correct (Go to loops folder)

This exercise proposes a few simple algorithms involving loops. The aim is to add the necessary loop
variants and invariants to prove them correct

### Question 1 Find_Int_Sqrt function in isqrt.adb

Add a suitable loop invariant and prove the correctness of the loop.

### Question 2 Find_Int_Sqrt function in isqrt.adb

Add a suitable loop variant and prove that the loop terminates.

### Question 3 Arrays_Max function in max.adb 

Add a suitable loop invariant and prove the correctness of the loop.

### Question 4 Arrays_Max function in max.adb 

Add a suitable loop variant and prove that the loop terminates.

### Question 5 Merge function in max.adb 

Add a suitable loop invariant and prove the correctness of the loop.

### Question 6 Merge function in max.adb 

Do we need a variant?
Provide your answer in answers.txt in the loops folder

## Part 3 (Go to lcp folder)

Follow the instructions given in questions.pdf in the lcp folder.

## Part 4 (Go to challenge) [Bonus]

### Question 1

Prove the abasence of runtime errors in Prove_The_Absence_Of_Run_Time_Errors function.


# Submission
Once you have done the exercises keeping the folder structure provided
create a zip file in place named student_day3.zip containing the same folder structure and files and submit the zip file on Moodle.

