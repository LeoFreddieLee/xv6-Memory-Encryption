#include "param.h"
#include "types.h"
#include "defs.h"
#include "x86.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "elf.h"

extern char data[];  
pde_t *kpgdir;  


void
seginit(void)
{
  struct cpu *c;


  c = &cpus[cpuid()];
  c->gdt[SEG_KCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, 0);
  c->gdt[SEG_KDATA] = SEG(STA_W, 0, 0xffffffff, 0);
  c->gdt[SEG_UCODE] = SEG(STA_X|STA_R, 0, 0xffffffff, DPL_USER);
  c->gdt[SEG_UDATA] = SEG(STA_W, 0, 0xffffffff, DPL_USER);
  lgdt(c->gdt, sizeof(c->gdt));
}

static pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
  pde_t *pde;
  pte_t *pgtab;

  pde = &pgdir[PDX(va)];
  if(*pde & PTE_P){

    pgtab = (pte_t*)P2V(PTE_ADDR(*pde));
  } else {
    if(!alloc || (pgtab = (pte_t*)kalloc()) == 0)
      return 0;

    memset(pgtab, 0, PGSIZE);

    *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
  }
  return &pgtab[PTX(va)];
}



static int
mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm)
{
  char *a, *last;
  pte_t *pte;

  a = (char*)PGROUNDDOWN((uint)va);
  last = (char*)PGROUNDDOWN(((uint)va) + size - 1);
  for(;;){
    if((pte = walkpgdir(pgdir, a, 1)) == 0)
      return -1;

    if (perm & PTE_E)
      *pte = pa | perm | PTE_E;
    else
      *pte = pa | perm | PTE_P;


    if(a == last)
      break;
    a += PGSIZE;
    pa += PGSIZE;
  }
  return 0;
}

// There is one page table per process, plus one that's used when
// a CPU is not running any process (kpgdir). The kernel uses the
// current process's page table during system calls and interrupts;
// page protection bits prevent user code from using the kernel's
// mappings.
//
// setupkvm() and exec() set up every page table like this:
//
//   0..KERNBASE: user memory (text+data+stack+heap), mapped to
//                phys memory allocated by the kernel
//   KERNBASE..KERNBASE+EXTMEM: mapped to 0..EXTMEM (for I/O space)
//   KERNBASE+EXTMEM..data: mapped to EXTMEM..V2P(data)
//                for the kernel's instructions and r/o data
//   data..KERNBASE+PHYSTOP: mapped to V2P(data)..PHYSTOP,
//                                  rw data + free physical memory
//   0xfe000000..0: mapped direct (devices such as ioapic)
//
// The kernel allocates physical memory for its heap and for user memory
// between V2P(end) and the end of physical memory (PHYSTOP)
// (directly addressable from end..P2V(PHYSTOP)).

// This table defines the kernel's mappings, which are present in
// every process's page table.
static struct kmap {
  void *virt;
  uint phys_start;
  uint phys_end;
  int perm;
} kmap[] = {
 { (void*)KERNBASE, 0,             EXTMEM,    PTE_W}, 
 { (void*)KERNLINK, V2P(KERNLINK), V2P(data), 0},     
 { (void*)data,     V2P(data),     PHYSTOP,   PTE_W}, 
 { (void*)DEVSPACE, DEVSPACE,      0,         PTE_W}, 
};


pde_t*
setupkvm(void)
{
  pde_t *pgdir;
  struct kmap *k;

  if((pgdir = (pde_t*)kalloc()) == 0)
    return 0;
  memset(pgdir, 0, PGSIZE);
  if (P2V(PHYSTOP) > (void*)DEVSPACE)
    panic("PHYSTOP too high");
  for(k = kmap; k < &kmap[NELEM(kmap)]; k++)
    if(mappages(pgdir, k->virt, k->phys_end - k->phys_start,
                (uint)k->phys_start, k->perm) < 0) {
      freevm(pgdir);
      return 0;
    }
  return pgdir;
}

void
kvmalloc(void)
{
  kpgdir = setupkvm();
  switchkvm();
}

void
switchkvm(void)
{
  lcr3(V2P(kpgdir));   
}

void
switchuvm(struct proc *p)
{
  if(p == 0)
    panic("switchuvm: no process");
  if(p->kstack == 0)
    panic("switchuvm: no kstack");
  if(p->pgdir == 0)
    panic("switchuvm: no pgdir");

  pushcli();
  mycpu()->gdt[SEG_TSS] = SEG16(STS_T32A, &mycpu()->ts,
                                sizeof(mycpu()->ts)-1, 0);
  mycpu()->gdt[SEG_TSS].s = 0;
  mycpu()->ts.ss0 = SEG_KDATA << 3;
  mycpu()->ts.esp0 = (uint)p->kstack + KSTACKSIZE;

  mycpu()->ts.iomb = (ushort) 0xFFFF;
  ltr(SEG_TSS << 3);
  lcr3(V2P(p->pgdir)); 
  popcli();
}


void
inituvm(pde_t *pgdir, char *init, uint sz)
{
  char *mem;

  if(sz >= PGSIZE)
    panic("inituvm: more than a page");
  mem = kalloc();
  memset(mem, 0, PGSIZE);
  mappages(pgdir, 0, PGSIZE, V2P(mem), PTE_W|PTE_U);
  memmove(mem, init, sz);
}

int
loaduvm(pde_t *pgdir, char *addr, struct inode *ip, uint offset, uint sz)
{
  uint i, pa, n;
  pte_t *pte;

  if((uint) addr % PGSIZE != 0)
    panic("loaduvm: addr must be page aligned");
  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walkpgdir(pgdir, addr+i, 0)) == 0)
      panic("loaduvm: address should exist");
    pa = PTE_ADDR(*pte);
    if(sz - i < PGSIZE)
      n = sz - i;
    else
      n = PGSIZE;
    if(readi(ip, P2V(pa), offset+i, n) != n)
      return -1;
  }
  return 0;
}


int
allocuvm(pde_t *pgdir, uint oldsz, uint newsz) 
{
  char *mem;
  uint a;

  if(newsz >= KERNBASE)
    return 0;
  if(newsz < oldsz)
    return oldsz;

  a = PGROUNDUP(oldsz);
  for(; a < newsz; a += PGSIZE){
    mem = kalloc();
    if(mem == 0){
      cprintf("allocuvm out of memory\n");
      deallocuvm(pgdir, newsz, oldsz);
      return 0;
    }
    memset(mem, 0, PGSIZE);
    if(mappages(pgdir, (char*)a, PGSIZE, V2P(mem), PTE_W|PTE_U) < 0){
      cprintf("allocuvm out of memory (2)\n");
      deallocuvm(pgdir, newsz, oldsz);
      kfree(mem);
      return 0;
    }
  }
  return newsz;
}


int
deallocuvm(pde_t *pgdir, uint oldsz, uint newsz)
{
  pte_t *pte;
  uint a, pa;

  if(newsz >= oldsz)
    return oldsz;

  a = PGROUNDUP(newsz);
  for(; a  < oldsz; a += PGSIZE){
    pte = walkpgdir(pgdir, (char*)a, 0);
    if(!pte)
      a = PGADDR(PDX(a) + 1, 0, 0) - PGSIZE;
    else if((*pte & (PTE_P | PTE_E)) != 0){
      pa = PTE_ADDR(*pte);
      if(pa == 0)
        panic("kfree");
      char *v = P2V(pa);
      kfree(v);
      *pte = 0;
    }
  }
  return newsz;
}


void
freevm(pde_t *pgdir)
{
  uint i;

  if(pgdir == 0)
    panic("freevm: no pgdir");
  deallocuvm(pgdir, KERNBASE, 0);
  for(i = 0; i < NPDENTRIES; i++){
    if(pgdir[i] & (PTE_P | PTE_E)){
      char * v = P2V(PTE_ADDR(pgdir[i]));
      kfree(v);
    }
  }
  kfree((char*)pgdir);
}


void
clearpteu(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);
  if(pte == 0)
    panic("clearpteu");
  *pte &= ~PTE_U;
}


pde_t*
copyuvm(pde_t *pgdir, uint sz)
{
  pde_t *d;
  pte_t *pte;
  uint pa, i, flags;
  char *mem;

  if((d = setupkvm()) == 0)
    return 0;
  for(i = 0; i < sz; i += PGSIZE){
    pa = PTE_ADDR(*pte);
    flags = PTE_FLAGS(*pte);
    if((mem = kalloc()) == 0)
      goto bad;
    memmove(mem, (char*)P2V(pa), PGSIZE);
    if(mappages(d, (void*)i, PGSIZE, V2P(mem), flags) < 0) {
      kfree(mem);
      goto bad;
    }
  }
  return d;

bad:
  freevm(d);
  return 0;
}


char*
uva2ka(pde_t *pgdir, char *uva)
{
  pte_t *pte;

  pte = walkpgdir(pgdir, uva, 0);

  if(((*pte & PTE_P) | (*pte & PTE_E)) == 0)
    return 0;
  if((*pte & PTE_U) == 0)
    return 0;
  return (char*)P2V(PTE_ADDR(*pte));
}


int
copyout(pde_t *pgdir, uint va, void *p, uint len)
{
  char *buf, *pa0;
  uint n, va0;

  buf = (char*)p;
  while(len > 0){
    va0 = (uint)PGROUNDDOWN(va);
    pa0 = uva2ka(pgdir, (char*)va0);
    if(pa0 == 0)
    {
 
      return -1;
    }
    n = PGSIZE - (va - va0);
    if(n > len)
      n = len;
    memmove(pa0 + (va - va0), buf, n);
    len -= n;
    buf += n;
    va = va0 + PGSIZE;
  }
  return 0;
}


char* translate_and_set(pde_t *pgdir, char *uva) {
  pte_t *pte;
  pte = walkpgdir(pgdir, uva, 0);


  if((*pte & PTE_P) == 0 && (*pte & PTE_E) == 0)
    return 0;

  if((*pte & PTE_E)) {
    return 0;
  }

  if((*pte & PTE_U) == 0)
    return 0;

  *pte = *pte | PTE_E;
  *pte = *pte & ~PTE_P;
  *pte = *pte & ~PTE_A;
  return (char*)P2V(PTE_ADDR(*pte));
}


void put_in_queue(char * virtual_addr, struct proc * p, pde_t* mypd) {

  int c=0;
  while(1)
  {
    if (p->list[c].used) {
    }else{
        p->list[c].va = virtual_addr;
        p->list[c].pte = walkpgdir(mypd, p->list[c].va, 0);
        p->list[c].used = 1;

        p->counter++;
        c = -256;
        break;
    }
    c++;
    if(c>CLOCKSIZE){
      break;
    }
  }
  
  if (c == -256) {
    //error case to be implemented 
  }else{
    c = 0;

    for (;;p->list_start = p->list_start->next) {
      pte_t * curr_pte = walkpgdir(mypd, p->list_start->va, 0);
      if (!(*curr_pte & PTE_A)) {
        mencrypt(p->list_start->va, 1);
        p->list_start->va = virtual_addr; 
        p->list_start->pte = walkpgdir(mypd, p->list_start->va, 0);
        break;
      } else {
        *curr_pte = *curr_pte & ~PTE_A;  
      }
    }
  }
}


int mdecrypt(char *virtual_addr) {



  //cprintf("p4Debug:  mdecrypt VPN %d, %p, pid %d\n", PPN(virtual_addr), virtual_addr, myproc()->pid);
  struct proc * p = myproc();
  pde_t* mypd = p->pgdir;
  pte_t * pte = walkpgdir(mypd, virtual_addr, 0);


    cprintf("%d\n", CLOCKSIZE);
    

  if (!pte || *pte == 0) {
    cprintf("p4Debug: walkpgdir failed\n");
    return -1;
  }
  cprintf("p4Debug: pte was %x\n", *pte);
  *pte = *pte & ~PTE_E;
  *pte = *pte | PTE_P;
  *pte = *pte | PTE_A;
  //cprintf("p4Debug: pte is %x\n", *pte);
  char * original = uva2ka(mypd, virtual_addr) + OFFSET(virtual_addr);
  //cprintf("p4Debug: Original in decrypt was %p\n", original);
  virtual_addr = (char *)PGROUNDDOWN((uint)virtual_addr);
  //cprintf("p4Debug: mdecrypt: rounded down va is %p\n", virtual_addr);

  char * kvp = uva2ka(mypd, virtual_addr);
  if (!kvp || *kvp == 0) {
    return -1;
  }
  char * slider = virtual_addr;
  for (int offset = 0; offset < PGSIZE; offset++) {
    *slider = *slider ^ 0xFF;
    slider++;
  }
  put_in_queue(virtual_addr, p, mypd);
  switchuvm(p);
  return 0;
}

int mencrypt(char *virtual_addr, int len) {

  //cprintf("p4Debug: mencrypt: %p %d\n", virtual_addr, len);
  struct proc * p = myproc();
  pde_t* mypd = p->pgdir;

  virtual_addr = (char *)PGROUNDDOWN((uint)virtual_addr);

  char * slider = virtual_addr;
  for (int i = 0; i < len; i++) { 
    char * kvp = uva2ka(mypd, slider);
    //cprintf("p4Debug: slider %p, kvp for err check is %p\n",slider, kvp);
    if (!kvp) {
      //cprintf("p4Debug: mencrypt: kvp = NULL\n");
      return -1;
    }
    slider = slider + PGSIZE;
  }

  slider = virtual_addr;
  for (int i = 0; i < len; i++) {
    //cprintf("p4Debug: mencryptr: VPN %d, %p\n", PPN(slider), slider);
    char * kvp = uva2ka(mypd, slider);
    //cprintf("p4Debug: kvp for encrypt stage is %p\n", kvp);
    pte_t * mypte = walkpgdir(mypd, slider, 0);
    //cprintf("p4Debug: pte is %x\n", *mypte);
    if (*mypte & PTE_E) {
      //cprintf("p4Debug: already encrypted\n");
      slider += PGSIZE;
      continue;
    }
    for (int offset = 0; offset < PGSIZE; offset++) {
      *slider = *slider ^ 0xFF;
      slider++;
    }
    char * kvp_translated = translate_and_set(mypd, slider-PGSIZE);
    if (!kvp_translated) {
      //cprintf("p4Debug: translate failed!");
      return -1;
    }
  }

  switchuvm(myproc());
  return 0;
}

int not_in_queue(pte_t * pte) {
  struct proc * p = myproc();
  int c = 0;
  while(1)
   {
    if (pte == walkpgdir(p->pgdir, p->list[c].va, 0)) { 
      return 0; 
      }
    c++;
    if(c>CLOCKSIZE){
      break;
    }
  }
  return 1;
}

int getpgtable(struct pt_entry* pt_entries, int num, int wsetOnly) {
  struct proc *curproc = myproc();
  pde_t *pgdir = curproc->pgdir;
  uint uva = 0;
  if (curproc->sz % PGSIZE == 0)
    uva = curproc->sz - PGSIZE;
  else 
    uva = PGROUNDDOWN(curproc->sz);

  int i = 0;
  for (;;uva -=PGSIZE)
  {
    
    pte_t *pte = walkpgdir(pgdir, (const void *)uva, 0);

    if (!(*pte & PTE_U) || !(*pte & (PTE_P | PTE_E)))
      continue;
    
    if (wsetOnly && not_in_queue(pte)) {
      continue;
    }

    pt_entries[i].pdx = PDX(uva);
    pt_entries[i].ptx = PTX(uva);
    pt_entries[i].ppage = *pte >> PTXSHIFT;
    pt_entries[i].present = *pte & PTE_P;
    pt_entries[i].writable = (*pte & PTE_W) > 0;
    pt_entries[i].encrypted = (*pte & PTE_E) > 0;
    pt_entries[i].ref = (*pte & PTE_A) > 0;
    i ++;
    if (uva == 0 || i == num) break;

  }

  return i;

}


int dump_rawphymem(char *physical_addr, char * buffer) {
  *buffer = *buffer;
  int retval = copyout(myproc()->pgdir, (uint) buffer, (void *) PGROUNDDOWN((int)P2V(physical_addr)), PGSIZE);
  if (retval)
    return -1;
  return 0;
}
