#include <iostream>

class telephone
{
public:
  void decrocher()
  {
    // FIXME-begin
    telephone_decrocher_impl();
    // FIXME-end
  }

private:
  // FIXME-begin
  virtual void telephone_decrocher_impl() = 0;
  // FIXME-end
};

class objet_mural
{
public:
  void decrocher()
  {
    // FIXME-begin
    objet_mural_decrocher_impl();
    // FIXME-end
  }

private:
  // FIXME-begin
  virtual void objet_mural_decrocher_impl() = 0;
  // FIXME-end
};

class telephone_mural: private objet_mural, private telephone
{
public:

  void decrocher_telephone()
  {
    // FIXME-begin
    telephone::decrocher();
    // FIXME-end
  }

  void decrocher_objet_mural()
  {
    // FIXME-begin
    objet_mural::decrocher();
    // FIXME-end
  }

private:
  // FIXME-begin
  virtual void telephone_decrocher_impl()
  {
    std::cout << "telephone_mural::telephone_decrocher_impl" << std::endl;
  }
  // FIXME-end

  // FIXME-begin
  virtual void objet_mural_decrocher_impl()
  {
    std::cout << "telephone_mural::objet_mural_decrocher_impl" << std::endl;
  }
  // FIXME-end
};


int main()
{
  telephone_mural* t = new telephone_mural();

  std::cout << "---" << std::endl;
  t->decrocher_telephone();

  std::cout << "---" << std::endl;
  t->decrocher_objet_mural();

  std::cout << "---" << std::endl;
  ((telephone*) t)->decrocher();

  std::cout << "---" << std::endl;
  ((objet_mural*) t)->decrocher();

  delete t;
}
