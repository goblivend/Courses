with Ada.Unchecked_Deallocation;

package body Ordered_Structures is

   ------------------
   -- Ordered_List --
   ------------------

   function New_Ordered_List return Ordered_List
   is (Data => Element_Lists.Empty_List);

   overriding function Contains
     (Self : Ordered_List; E : Element) return Boolean
   is (Self.Data.Contains (E));

   overriding function Is_Empty (Self : Ordered_List) return Boolean
   is (Self.Data.Is_Empty);

   overriding function Get_Elements (Self : Ordered_List) return Element_Array
   is
      Res : Element_Array (1 .. Integer (Self.Data.Length));
      Cur : Element_Lists.Cursor := Self.Data.First;
   begin
      for I in Res'Range loop
         pragma Assert (Element_Lists.Has_Element (Cur));
         Res (I) := Element_Lists.Element (Cur);
         Element_Lists.Next (Cur);
      end loop;

      return Res;
   end Get_Elements;

   overriding procedure Insert (Self : in out Ordered_List; E : Element) is
      Cur : Element_Lists.Cursor := Self.Data.First;
   begin
      while Element_Lists.Has_Element (Cur)
        and then Element_Lists.Element (Cur) <= E loop
         Element_Lists.Next (Cur);
      end loop;

      Self.Data.Insert (Before => Cur, New_Item => E);
   end Insert;

   overriding procedure Delete (Self : in out Ordered_List; E : Element) is
      Pos : Element_Lists.Cursor := Self.Data.Find (E);
   begin
      Self.Data.Delete (Pos);
   end Delete;


   ---------
   -- BST --
   ---------

   procedure Free_Node is
     new Ada.Unchecked_Deallocation (BST_Node, BST_Node_Ptr);

   function New_BST return BST
   is (Root => null);

   overriding function Get_Elements (Self : BST) return Element_Array is
      function Get_Elements (N : BST_Node_Ptr) return Element_Array
      is (if N = null then (1 .. 0 => 0)
          else Get_Elements (N.Left) & N.Value & Get_Elements (N.Right));

   begin
      return Get_Elements (Self.Root);
   end Get_Elements;

   overriding function Contains (Self : BST; E : Element) return Boolean is
      function Contains (N : BST_Node_Ptr; E : Element) return Boolean
      is (N /= null
          and then (N.Value = E
                    or else Contains (N.Left, E)
                    or else Contains (N.Right, E)));

   begin
      return Contains (Self.Root, E);
   end Contains;

   function Is_Empty (Self : BST) return Boolean is (Self.Root = null);

   overriding procedure Insert (Self : in out BST; E : Element) is
      Cur : BST_Node_Ptr := Self.Root;
   begin
      if Self.Root = null then
         Self.Root := new BST_Node'(E, null, null);
         return;
      end if;

      loop
         if E <= Cur.Value then
            if Cur.Left = null then
               Cur.Left := new BST_Node'(E, null, null);
               exit;
            else
               Cur := Cur.Left;
            end if;
         else
            if Cur.Right = null then
               Cur.Value := E;
               exit;
            else
               Cur := Cur.Right;
            end if;
         end if;
      end loop;
   end Insert;

   overriding procedure Delete (Self : in out BST; E : Element) is

      function Find_Predecessor_And_Remove
        (N : in out BST_Node_Ptr) return Element is
      begin
         if N.Right = null then
            declare
               Tmp : BST_Node_Ptr := N.Left;
               Res : Element := N.Value;
            begin

               Free_Node (N);
               N := Tmp;

               return Res;
            end;
         else
            --  Go to the right most node
            return Find_Predecessor_And_Remove (N.Right);
         end if;
      end Find_Predecessor_And_Remove;

      procedure Delete_Subtree_Root (N : in out BST_Node_Ptr; E : Element) is
         procedure Son_Replace_Father (Father, Son : in out BST_Node_Ptr) is
         begin
            Free_Node (Father);
            Father := Son;
         end Son_Replace_Father;

      begin
         if N.Left = null and then N.Right = null then
            --  Leaf : just delete the node
            Free_Node (N);
         elsif N.Right = null then
            Son_Replace_Father (N, N.Left);
         elsif N.Left = null then
            Son_Replace_Father (N, N.Right);
         else
            N.Value := Find_Predecessor_And_Remove (N.Left);
         end if;
      end Delete_Subtree_Root;

      procedure Delete (N : in out BST_Node_Ptr; E : Element) is
      begin
         if N.Value = E then
            Delete_Subtree_Root (N, E);
         elsif E < N.Value then
            Delete (N.Left, E);
         else
            Delete (N.Right, E);
         end if;
      end Delete;
   begin
      Delete (Self.Root, E);
   end Delete;

end Ordered_Structures;
