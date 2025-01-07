(* The goal of this exam is to prove the equivalence:

a = 1 /\ b = 1 <-> a * b = 1.

and then a more general verison on lists.

It is split into 8 propositions you will have to prove.

Your goal is to replace the "admit. Admitted." part by an actual
proof, validated by Qed.

You MUST respect the following instructions

- only tactics from the cheatsheet are authorized
- lemmas from the standard library are forbidden.
- do not change the propositions q1, q2 ...
- do not load any third party library
- do not add load any module (e.g, Arith, ...) (the ones you will need are already loaded)
- remove printing (Show, Locate, etc) before submitting

You can:
- add intermediate lemmas *if you prove them* (you can not admit them)
- use a result from a previous question even if it is admitted

Failure to comply to one or more instructions can lead to grade of 0
of the whole exercise. Syntactically invalid files and files that fails to
interpret correctly will be graded 0.
*)

Proposition q0 :
  forall a:nat, a + 0 = a.
Proof.
  intros.
  induction a.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.

Proposition q1 :
  forall a:nat, a * 0 = 0.
Proof.
  intros.
  induction a.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.

Proposition q2 :
  forall a b: nat, a + S b = S (a + b).
Proof.
  intros.
  induction a.
  - simpl.
     reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.

Proposition q3 :
  forall a b :nat, a = 1 /\ b = 1 -> a * b = 1.
Proof.
  intros.
  destruct H.
  rewrite H.
  rewrite H0.
  simpl.
  reflexivity.
Qed.

(* Hint 1: reasonning on the case b=0, b>=1 can be useful *)
(* Hint 2: the 'discriminate H' tactic makes it possible to solve a goal
as soon as there is an equality between two different constructor in
the hypothesis 'H'.*)
Proposition q4 :
  forall a b :nat, S (S a) * b = 1 -> False.
Proof.
  intros.
  induction b.
  - rewrite q1 in H.
    discriminate H.
  - simpl in H.
    rewrite q2 in H.
    discriminate H.
Qed.


(* Hint: reasonning on the three cases a=0, a=1, a>=2, should help you *)
Proposition q5 :
  forall a b :nat, a * b = 1 -> a = 1 /\ b = 1.
Proof.
  intros.
  induction a. (*Induction a=0, a>=1*)
  - simpl in H.
    discriminate H.
  - destruct a. (*Induction a=1, a>=2*)
    + simpl in H.
      rewrite q0 in H. (* b+0=1 => b=1*)
      split. (* Need to split `/\` *)
      * reflexivity.
      * assumption.
    + apply q4 in H.
      contradiction.
Qed.

Require Import List.
Require Import Setoid.
Import ListNotations.

(* Builds the proposition : all elements in l are equal to 1 *)
Fixpoint f1 (l:list nat) : Prop :=
  match l with
  | [] => True
  | h::t => h = 1 /\ f1 t
  end.

(* [List.fold_right Nat.mul 1 l] computes the product of all the integer in l, using the initial value 1. *)
Proposition q6 :
  forall l,
    List.fold_right Nat.mul 1 l = 1 -> f1 l.
Proof.
  intros.
  induction l.
  - simpl.
    reflexivity.
  - simpl in H.
    apply q5 in H.
    destruct H.
    split.
    + assumption.
    + apply IHl.
      assumption.
Qed.

Proposition q7 :
  forall l, f1 l -> List.fold_right Nat.mul 1 l = 1.
Proof.
  intros.
  induction l.
  - simpl.
    reflexivity.
  - simpl.
    destruct H.
    apply IHl in H0.
    rewrite H0.
    rewrite H.
    simpl.
    reflexivity.
Qed.


Proposition q8 :
  forall l,
    List.fold_right Nat.mul 1 l = 1 <-> f1 l.
Proof.
  intros.
  split.
  - apply q6.
  - apply q7.
Qed.
