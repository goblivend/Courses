package body Geometry is
   function Squared_Norm (Self : Coordinates) return Float
   is (Self.X * Self.X + Self.Y * Self.Y);

   function Squared_Norm (Self : Vector) return Float
   is (Self.X * Self.X - Self.Y * Self.Y);
end Geometry;
