Proposition modus_ponens :
  forall A B : Prop, (A -> B) -> A -> B.

Proof.
  intros.
  apply H.
  assumption.
Qed.

Proposition trans_impl :
  forall A B C: Prop, (A -> B) -> (B -> C) -> (A -> C).

Proof.
  intros.
  apply H0.
  apply H.
  assumption.
Qed.

Proposition modus_tollens :
  forall A B: Prop, (A -> B) -> ~B -> ~A.


Proof.
  intros.
  intro.
  apply H0.
  apply H.
  assumption.
Qed.


Proposition a_or_b_false_left:
  forall A B : Prop, (( A \/ B) -> False) -> (A -> False).

Proof.
  intros.
  apply H.
  left.
  assumption.
Qed.

Proposition a_or_b_false_right:
  forall A B : Prop, (( A \/ B) -> False) -> (B -> False).

Proof.
  intros.
  apply H.
  right.
  assumption.
Qed.

Proposition de_morgan_bool_1 :
  forall a b:bool, negb (orb a b) = andb (negb a) (negb b).

Proof.
  intros.
  destruct a.
  - destruct b.
    simpl.
    reflexivity.
    simpl.
    reflexivity.
  - destruct b.
    simpl.
    reflexivity.
    simpl.
    reflexivity.
Qed.

Proposition de_morgan_bool_2 :
  forall a b:bool, negb (andb a b) = orb (negb a) (negb b).

Proof.
  intros.
  destruct a.
  - destruct b.
    reflexivity.
    reflexivity.
  - destruct b.
    reflexivity.
    reflexivity.
Qed.


Proposition de_morgan_1 :
  forall P Q, ~(P \/ Q) -> ~P /\ ~Q.

Proof.
  unfold "~".
  intros.
  split.
  - intro.
    apply H.
    left.
    apply H0.
  - intro.
    apply H.
    right.
    apply H0.
Qed.

Proposition de_morgan_2 :
  forall P Q, ~P /\ ~Q -> ~(P \/ Q).

Proof.
  unfold "~".  
  intros P Q.
  intro.
  destruct H.
  intro.
  destruct H1.
  - apply H.
    assumption.
  - apply H0.
    assumption.
Qed.


Proposition de_morgan_1_2 :
  forall P Q, ~P /\ ~Q <-> ~(P \/ Q).

Proof.
  intros.
  split.
  - apply de_morgan_2.
  - apply de_morgan_1.
Qed.

Proposition excluded_middle_irrefutable:
  forall P:Prop, ~ ~ (P \/ ~ P).

Proof.
  intros.
  intro.
  apply de_morgan_1 with (P:=P) (Q:=~P).
  
  
Qed.
