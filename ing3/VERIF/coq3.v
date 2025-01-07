  Require Import List.
  Import ListNotations.

  Proposition concat_left_nil :
    forall (A : Set) (l : list A), (nil ++ l) = l.
  Proof.
    intros.
    simpl.
    reflexivity.
  Qed.

  Proposition concat_right_nil :
    forall (A : Set) (l : list A), (l ++ nil) = l.
  Proof.
    intros.
    induction l.
    - simpl.
      reflexivity.
    - simpl.
      rewrite IHl.
      reflexivity.
  Qed.

  Proposition associative_concat :
    forall (A : Set) (l1 l2 l3 : list A), (l1 ++ l2) ++ l3 = l1 ++ (l2 ++ l3).
  Proof.
    intros.
    induction l1.
    - simpl.
      reflexivity.
    - simpl.
      rewrite IHl1.
      reflexivity.
  Qed.

  Fixpoint length {A:Set} (l : list A) : nat :=
    match l with
    | [] => 0
    | _ :: l1 => 1 + length l1
  end.

  Proposition concat_length_sum : forall (A:Set) (xs ys: list A), length
  (xs ++ ys) = length xs + length ys.
  Proof.
    intros.
    induction xs.
    - simpl.
      reflexivity.
    - simpl.
      rewrite IHxs.
      reflexivity.
  Qed.

  Fixpoint rev {A:Set} (l : list A) :=
    match l with
    | [] => []
    | a :: l1 => rev l1 ++ [a]
    end.

  Lemma sum_1 : forall (A : Set) (l : list A) (a : A), length (l ++ [a]) = S (length l).
  Proof.
    intros.
    induction l.
    - simpl.
      reflexivity.
    - simpl.
      rewrite IHl.
      reflexivity.
  Qed.

  Proposition rev_preserve_length : forall (A : Set) (l : list A), length (rev l) = length l.
  Proof.
    intros.
    induction l.
    - simpl.
      reflexivity.
    - simpl.
      rewrite sum_1 with (l := rev l).
      rewrite IHl.
      reflexivity.
  Qed.

  Fixpoint nth {A:Set} (i : nat) (xs:(list A)): (option A) :=
    match xs with
    | [] => None
    | a :: l => match i with
                | 0 => Some a
                | S(n') => nth n' l
                end
    end.

  Proposition nth_len_app1 :
  forall (A:Set) (l1 l2 : list A),
  nth (length l1) (l1 ++ l2) = nth 0 l2.
  Proof.
    intros.
    induction l1.
    - simpl ([] ++ l2).
      simpl (length []).
      reflexivity.
    - simpl (nth (length (a :: l1)) ((a :: l1) ++ l2)) .
      rewrite IHl1.
      reflexivity.
  Qed.

  Lemma append_1 : forall (A : Set) (l : list A) (a : A), length (a :: l) = S (length l).
  Proof.
    intros.
    simpl.
    reflexivity.
  Qed.

  (* Proposition nth_len_app2 :
  forall (A:Set) (l1 l2: list A) (i: nat),
  (i < length l1) -> nth i (l1 ++ l2) = nth i l1.
  Proof.
    induction l1.
    - simpl.
      intros.
      apply PeanoNat.Nat.nlt_0_r in H.
      contradiction.
    - intros.
      destruct i.
      -- simpl.
        reflexivity.
      -- simpl.
        rewrite append_1 in H.
        apply PeanoNat.lt_S_n in H.
        apply IHl1.
        assumption.
  Qed. *)

  Proposition rev_distributive : forall (A : Set) (l1 l2 : list A),
    rev (l1 ++ l2) = rev l2 ++ rev l1.
  Proof.
    intros.
    induction l1.
    - simpl.
      rewrite concat_right_nil.
      reflexivity.
    - simpl.
      rewrite IHl1.
      rewrite associative_concat.
      reflexivity.
  Qed.

  Lemma rev_involution :
  forall (A:Set) (xs ys: list A),
  rev ((rev xs) ++ ys) = rev ys ++ xs.
  Proof.
    induction xs.
    - simpl.
      intros.
      rewrite concat_right_nil.
      reflexivity.
    - simpl.
      intros.
      rewrite associative_concat.
      rewrite IHxs.
      rewrite rev_distributive.
      simpl.
      rewrite associative_concat.
      simpl.
      reflexivity.
  Qed.

  Lemma rev_involutive:
  forall (A:Set) (l: list A), rev (rev l) = l.
  Proof.
    intros.
    assert (rev l = rev l ++ []).
    - rewrite concat_right_nil.
      reflexivity.
    - rewrite H.
      rewrite rev_involution with (ys := []).
      simpl.
      reflexivity.
  Qed.

  Lemma rev_involutive2:
  forall (A:Set) (l: list A), rev (rev l) = l.
  Proof.
    intros.
    induction l.
    - simpl.
      reflexivity.
    - simpl.
      rewrite rev_distributive.
      simpl.
      rewrite IHl.
      reflexivity.
  Qed.
