pragma Extensions_Allowed (On);

with Ada.Streams.Stream_IO; use Ada.Streams.Stream_IO;
with Ada.Text_IO;          -- use Ada.Text_IO;
with Ada.Command_Line;      use Ada.Command_Line;

procedure Elf_Reader is
   type Format is (Bit32, Bit64);
   for Format use (Bit32 => 1, Bit64 => 2);
   type Endian is (Little_Endian, Big_Endian);
   for Endian use (Little_Endian => 1, Big_Endian => 2);
   type Osabi is
     (SystemV,
      HPUX,
      NetBSD,
      Linux,
      GNU_Hurd,
      Solaris,
      AIX,
      IRIX,
      FreeBSD,
      Tru64,
      Novell,
      OpenBSD,
      OpenVMS,
      NonStop,
      AROS,
      FenixOS,
      CloudABI,
      Stratus);
   for Osabi use
     (SystemV  => 0,
      HPUX     => 1,
      NetBSD   => 2,
      Linux    => 3,
      GNU_Hurd => 4,
      Solaris  => 6,
      AIX      => 7,
      IRIX     => 8,
      FreeBSD  => 9,
      Tru64    => 10,
      Novell   => 11,
      OpenBSD  => 12,
      OpenVMS  => 13,
      NonStop  => 14,
      AROS     => 15,
      FenixOS  => 16,
      CloudABI => 17,
      Stratus  => 18);

   type MAGIC_NB is mod 256;
   type MAG is array (0 .. 3) of MAGIC_NB;
   type ELF_Ident is record
      EI_MAG        : MAG;
      EI_CLASS      : Format;
      EI_DATA       : Endian;
      EI_VERSION    : Integer range 0 .. 255;
      EI_OSABI      : Osabi;
      EI_ABIVERSION : Integer range 0 .. 255;
      -- EI_PAD : String (7);
   end record;
   for ELF_Ident use
     record
       EI_MAG at 0 range 0 .. 31;
       EI_CLASS at 4 range 0 .. 7;
       EI_DATA at 5 range 0 .. 7;
       EI_VERSION at 6 range 0 .. 7;
       EI_OSABI at 7 range 0 .. 7;
       EI_ABIVERSION at 8 range 0 .. 7;
       --   EI_PAD at 9 range 0 .. 127;
     end record;
   type ELF_Header is record
      e_ident   : ELF_Ident;
      e_type    : Integer range 0 .. 65535;
      e_machine : Integer range 0 .. 65535;
   end record;
   for ELF_Header use
     record
       e_ident at 0 range 0 .. 127;
       e_type at 16 range 0 .. 15;
       e_machine at 18 range 0 .. 15;
     end record;

   File   : File_Type;
   S      : Stream_Access;
   Header : ELF_Header;
begin
   if Argument_Count /= 1 then
      Ada.Text_IO.Put_Line ("Usage: elf_reader path/to/my/elf");
      return;
   end if;

   Open (File, In_File, Argument (1));
   S := Stream (File);
   ELF_Header'Read (S, Header);
   Close (File);

   Ada.Text_IO.Put_Line (Header'Image);
end Elf_Reader;
