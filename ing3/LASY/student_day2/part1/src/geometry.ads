package Geometry is
   type Coordinates is record
      X, Y : Float;
   end record;

   function Squared_Norm (Self : Coordinates) return Float
   with Post => Squared_Norm'Result >= 0.0;

   type Vector is new Coordinates;

   function Squared_Norm (Self : Vector) return Float;

end Geometry;
