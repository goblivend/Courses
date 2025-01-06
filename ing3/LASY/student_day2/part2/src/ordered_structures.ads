with Ada.Containers.Indefinite_Doubly_Linked_Lists;

package Ordered_Structures is
   type Element is new Integer;
   type Element_Array is array (Positive range <>) of Element;

   function Is_Ordered (A : Element_Array) return Boolean
   is (for all I in A'First + 1 .. A'Last => A (I) >= A (I - 1));

   function Same_Plus_E
     (Src, Target : Element_Array; E : Element) return Boolean
   is (for some I in Target'Range
       => Target (I) = E
       and then Target (Target'First .. I - 1) = Src (Src'First .. I - 1)
       and then Target (I + 1 .. Target'Last) = Src (I .. Src'Last));

   -----------------------
   -- Ordered_Structure --
   -----------------------

   -- Elements in ascending order
   type Ordered_Structure is interface
   with Type_Invariant'Class => Is_Ordered (Get_Elements (Ordered_Structure));

   function Get_Elements (Self : Ordered_Structure) return Element_Array
   is abstract;

   function Contains (Self : Ordered_Structure; E : Element) return Boolean
   is abstract;

   function Is_Empty (Self : Ordered_Structure) return Boolean is abstract;

   procedure Insert (Self : in out Ordered_Structure; E : Element) is abstract
   with
     Post'Class =>
       Same_Plus_E (Get_Elements (Self)'Old, Get_Elements (Self), E);

   procedure Delete (Self : in out Ordered_Structure; E : Element) is abstract
   with
     Pre'Class => Contains (Self, E),
     Post'Class =>
       Same_Plus_E (Get_Elements (Self), Get_Elements (Self)'Old, E);

   ------------------
   -- Ordered_List --
   ------------------

   type Ordered_List is new Ordered_Structure with private;

   overriding
   function Get_Elements (Self : Ordered_List) return Element_Array;

   overriding
   function Contains (Self : Ordered_List; E : Element) return Boolean;

   overriding
   function Is_Empty (Self : Ordered_List) return Boolean;

   overriding
   procedure Insert (Self : in out Ordered_List; E : Element);

   overriding
   procedure Delete (Self : in out Ordered_List; E : Element);

   function New_Ordered_List return Ordered_List
   with Post => New_Ordered_List'Result.Is_Empty;

   ---------
   -- BST --
   ---------

   type BST is new Ordered_Structure with private;

   overriding
   function Get_Elements (Self : BST) return Element_Array;

   overriding
   function Contains (Self : BST; E : Element) return Boolean;

   overriding
   function Is_Empty (Self : BST) return Boolean;

   overriding
   procedure Insert (Self : in out BST; E : Element);

   overriding
   procedure Delete (Self : in out BST; E : Element);

   function New_BST return BST
   with Post => New_BST'Result.Is_Empty;

private

   package Element_Lists is new
     Ada.Containers.Indefinite_Doubly_Linked_Lists (Element, "=");

   type Ordered_List is new Ordered_Structure with record
      Data : Element_Lists.List;
   end record;

   type BST_Node;
   type BST_Node_Ptr is access BST_Node;

   type BST_Node is record
      Value       : Element;
      Left, Right : BST_Node_Ptr;
   end record;

   type BST is new Ordered_Structure with record
      Root : BST_Node_Ptr;
   end record;

end Ordered_Structures;
