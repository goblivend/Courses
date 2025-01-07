Proposition plus_n_0 :
  forall n: nat, n+0=n.
Proof.
  intros.
  induction n.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHn.
    reflexivity.
 Qed.

Proposition double_is_plus:
  forall n : nat, n + n = 2 * n.
Proof.
  intros.
  induction n.
  - simpl.
    reflexivity.
  - simpl.
    rewrite plus_n_0.
    reflexivity.
Qed.

Proposition add_succ_l :
  forall n m: nat, S n + m = S (n + m).
Proof.
  intros.
  simpl.
  reflexivity.
Qed.

Proposition add_succ_r :
  forall n m: nat, n + S m = S (n + m).
Proof.
  intros.
  induction n.
  - simpl.
    reflexivity.
  - simpl.
    rewrite IHn.
    reflexivity.
Qed.

Fixpoint even(n : nat) : Prop :=
  match n with
  | 0 => True
  | S 0 => False
  | S (S n) => even n end.

Proposition one_of_two_succ_is_even:
  forall n : nat, (even n) \/ (even (S n)).
Proof.
  intros.
  induction n.
  - simpl.
    left.
    apply I.
  - destruct IHn.
    -- simpl.
       right.
       assumption.
    -- left.
       assumption.
Qed.

Proposition but_not_both :
  forall n : nat, even n -> not (even (S n)).
Proof.
  intros.
  induction n.
  - simpl.
    unfold "~".
    intro.
    contradiction.
  - unfold "~".
    simpl.
    intro.
    apply IHn in H0.
    contradiction.
Qed.

Proposition double_is_even:
  forall n : nat, even (n*2).
Proof.
  intros.
  induction n.
  - simpl.
    apply I.
  - simpl.
    assumption.
Qed.

Proposition succ_double_is_odd :
  forall n : nat, ~(even (S (n*2))).
Proof.
  intros.
  assert (even (n * 2)).
  simpl.
  apply double_is_even.
  apply but_not_both.
  assumption.
Qed.

Proposition pair_induction :
  forall (P : nat -> Prop),
  P 0 -> P 1 -> (forall n, P n -> P (S n) -> P (S (S n))) ->
  forall x, P x.
Proof.
  intros.
  assert  (P x /\ P(S x)).
  induction x.
  - split.
    -- assumption.
    -- assumption.
  - split.
    -- destruct IHx.
       assumption.
    -- destruct IHx.
       apply H1.
       assumption.
       assumption.
  - destruct H2.
    assumption.
Qed.


Proposition even_sum :
  forall n m: nat, even n -> even m -> even (n + m). 
Proof.
  intros.
  induction n using pair_induction.
  - simpl.
    assumption.
  - simpl.
    contradiction.
  - simpl.
    apply IHn.
    simpl in H.
    assumption.
Qed.

Definition mystery (f : nat -> nat) : Prop :=
  exists t, t > 0 /\ forall x, f x = f (x+t).

Definition null_function (n : nat) : nat :=
  0.

Proposition null_is_mystery :
  mystery(null_function).
Proof.
  unfold mystery.
  exists 1.
  split.
  - unfold ">"; unfold "<".
    Search (_ <= _).
    apply le_n.
  - intros.
    unfold null_function.
    reflexivity.
Qed.

Definition is_inverse (f g : nat -> nat) : Prop :=
  forall n : nat, f (g n) = n.

Definition remove_one_or_give_0 (n : nat) : nat :=
  match n with
  | 0 => 0
  | S n => n end.

Proposition remove_one_is_s_inverse :
  is_inverse remove_one_or_give_0 S.
Proof.
  unfold is_inverse.
  intros.
  unfold remove_one_or_give_0.
  reflexivity.
Qed.