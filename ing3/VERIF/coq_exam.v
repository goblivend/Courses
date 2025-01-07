(* The goal of this exam is to prove the equality:

a * b = b * a.

It is split into 7 intermediary propositions you will have to prove.

Your goal is to replace the "admit. Admitted." part by an actual
proof, validated by Qed.

you MUST respect the following instructions

- only tactics from the cheatsheet are authorized
- lemmas from the standard library are forbidden.
- do not change the propositions q1, q2 ...
- do not add any other propositions to this file
- do not load any third party library
- do not load any module such as Arith, Lists ...
- remove printing (Show, Locate, etc) before submitting

Failure to comply to one or more instructions can lead to grade of 0
of the whole exercise. Syntactically invalid files will be graded 0 *)

Lemma q1 :
  forall a b c: nat,
    a + (b + c) = (a + b) + c.
Proof.
  intros.
  induction a.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.


Lemma q2 :
  forall a : nat,
    a + 0 = a.
Proof.
  intros.
  induction a.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.

Lemma q3 :
  forall a b : nat,
    S (a + b) = a + S b.
Proof.
  intros.
  induction a.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHa.
    reflexivity.
Qed.

Lemma q4 :
  forall a b : nat,
    a + b = b + a.
Proof.
   intros.
   induction a.
   - simpl.
     rewrite q2.
     reflexivity.
   - simpl.
     rewrite IHa.
     rewrite q3.
     reflexivity.
Qed.

Lemma q5 :
  forall a : nat,
    a * 0 = 0.
Proof.
   intros.
   induction a.
   - simpl.
     reflexivity.
   - simpl.
     rewrite IHa.
     reflexivity.
Qed.

Lemma q6 :
  forall n m: nat,
    m * S n = m + m * n.
Proof.
   intros.
   induction m.
   - simpl.
     reflexivity.
   - simpl.
     rewrite IHm.
     rewrite q1.
     rewrite q4 with (a := n).
     rewrite q1.
     reflexivity.
Qed.

Lemma q7 : forall n m : nat,  m*n = n*m.
Proof.
   intros.
   induction m.
   - simpl.
     rewrite q5.
     reflexivity.
   - simpl.
     rewrite q6.
     rewrite IHm.
     reflexivity.
Qed.