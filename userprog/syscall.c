#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include <string.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "threads/flags.h"
#include "threads/palloc.h"
#include "userprog/gdt.h"
#include "userprog/process.h"
#include "intrinsic.h"
#include "filesys/filesys.h"
#include "filesys/file.h"
#include "filesys/inode.h"
#include "vm/vm.h"

void syscall_entry(void);
void syscall_handler(struct intr_frame *);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081			/* Segment selector msr */
#define MSR_LSTAR 0xc0000082		/* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

#define STDIN 1
#define STDOUT 2

void halt(void);
void exit(int status);
tid_t fork(const char *thread_name, struct intr_frame *if_);
int exec(const char *file);
int wait(tid_t pid);
bool create(const char *file, unsigned initial_size);
bool remove(const char *file);
int open(const char *file);
int filesize(int fd);
int read(int fd, void *buffer, unsigned size);
int write(int fd, const void *buffer, unsigned size);
void seek(int fd, unsigned pos);
unsigned tell(int fd);
void close(int fd);

void *mmap(void *addr, size_t length, int writable, int fd, off_t offset);
void munmap(void *addr);

int dup2(int oldfd, int newfd);

void
syscall_init (void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			  FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);

	lock_init(&file_lock);
}

struct page *check_address(uint64_t addr)
{
	if (is_kernel_vaddr(addr) || addr == NULL)
		exit(-1);

	return spt_find_page(&thread_current()->spt, addr);
}

void check_valid_buffer(void *buffer, unsigned size, void *rsp, bool to_write)
{
	uintptr_t start_page = pg_round_down(buffer);
	uintptr_t end_page = pg_round_down(buffer + size - 1);

	if (!(buffer <= USER_STACK && buffer >= rsp))
	{
		for (; start_page <= end_page; start_page += PGSIZE)
		{
		struct page *page = check_address(start_page);
		if (page == NULL)
			exit(-1);

		if (to_write == true && page->writable == false)
			exit(-1);
		}
	}
}

/* The main system call interface */
void syscall_handler(struct intr_frame *f)
{
	thread_current()->user_rsp = f->rsp;
	switch (f->R.rax)
		{
		case SYS_HALT:
			halt();
			break;

		case SYS_EXIT:
			exit(f->R.rdi);
			break;

		case SYS_FORK:
			check_address(f->R.rdi);

			f->R.rax = fork(f->R.rdi, f);
			break;

		case SYS_EXEC:
			check_address(f->R.rdi);

			if (exec(f->R.rdi) < 0)
				exit(-1);
			break;

		case SYS_WAIT:
			f->R.rax = wait(f->R.rdi);
			break;

		case SYS_CREATE:
			check_address(f->R.rdi);

			f->R.rax = create(f->R.rdi, f->R.rsi);
			break;

		case SYS_REMOVE:
			check_address(f->R.rdi);

			f->R.rax = remove(f->R.rdi);
			break;

		case SYS_OPEN:
			check_address(f->R.rdi);

			f->R.rax = open(f->R.rdi);
			break;

		case SYS_FILESIZE:
			f->R.rax = filesize(f->R.rdi);
			break;

		case SYS_READ:
			check_valid_buffer(f->R.rsi, f->R.rdx, f->rsp, 1);

			f->R.rax = read(f->R.rdi, f->R.rsi, f->R.rdx);
			break;

		case SYS_WRITE:
			check_valid_buffer(f->R.rsi, f->R.rdx, f->rsp, 0);

			f->R.rax = write(f->R.rdi, f->R.rsi, f->R.rdx);
			break;

		case SYS_SEEK:
			seek(f->R.rdi, f->R.rsi);
			break;

		case SYS_TELL:
			f->R.rax = tell(f->R.rdi);
			break;

		case SYS_CLOSE:
			close(f->R.rdi);
			break;

		case SYS_MMAP:
			f->R.rax = mmap(f->R.rdi, f->R.rsi, f->R.rdx, f->R.r10, f->R.r8);
			break;

		case SYS_MUNMAP:
			check_address(f->R.rdi);

			munmap(f->R.rdi);
			break;

		case SYS_DUP2:
			f->R.rax = dup2(f->R.rdi, f->R.rsi);
			break;
			
		default:
			exit(-1);
			break; 
	}
}

void
halt(void)
{
	power_off();
}


void 
exit(int status)
{
	thread_current()->exit_status = status;
	printf("%s: exit(%d)\n", thread_name(), thread_current()->exit_status);

	thread_exit();
}

tid_t fork(const char *thread_name, struct intr_frame *if_)
{
	return process_fork(thread_name, if_);
}

int 
exec(const char *file)
{

	char *fn_copy = palloc_get_page(PAL_ZERO);
	if (fn_copy == NULL)
		return -1;

	strlcpy(fn_copy, file, PGSIZE);

	if (process_exec(fn_copy) < 0)
		return -1;

	return 0;
}

int
wait(tid_t pid)
{
	return process_wait(pid);
}

bool 
create(const char *file, unsigned initial_size)
{
	lock_acquire(&file_lock);
	bool succ = filesys_create(file, initial_size);
	lock_release(&file_lock);

	return succ;
}

bool 
remove(const char *file)
{
	return filesys_remove(file);
}

int 
open(const char *file)
{
	lock_acquire(&file_lock);
	struct file *f = filesys_open(file);
	lock_release(&file_lock);

	if (f == NULL)
		return -1;

	int fd = process_add_file(f);
	if (fd == -1)
		close(f);

	return fd;
}

int 
filesize(int fd)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	return file_length(f);
}

int 
read(int fd, void *buffer, unsigned size)
{
	struct file *f = process_get_file(fd);
	if (f == NULL || f == STDOUT)
		return -1;

	int read_result;

	if (f == STDIN)
	{
		for (read_result = 0; read_result < size; read_result++)
		{
			char key = input_getc();
			*(char *)buffer = key;
			(char *)buffer++;

			if (key == '\0')
				break;
		}
	}
	else
	{
		lock_acquire(&file_lock);
		read_result = file_read(f, buffer, size);
		lock_release(&file_lock);
	}

	return read_result;
}

int 
write(int fd, const void *buffer, unsigned size)
{
	struct file *f = process_get_file(fd);
	if (f == NULL || f == STDIN)
		return -1;

	int write_result;

	if (f == STDOUT)
	{
		putbuf(buffer, size);
		write_result = size;
	}
	else
	{
		if (inode_is_dir(f->inode) == INODE_DIR)
			return -1;

		lock_acquire(&file_lock);
		write_result = file_write(f, buffer, size);
		lock_release(&file_lock);
	}

	return write_result;
}

void 
seek(int fd, unsigned pos)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	file_seek(f, pos);
}

unsigned tell(int fd)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	return file_tell(f);
}

void 
close(int fd)
{
	struct thread *curr = thread_current();
	struct file *f = process_get_file(fd);
	if (f == NULL)
		return;

	if (f == STDIN)
		curr->stdin_cnt--;
	else if (f == STDOUT)
		curr->stdout_cnt--;
	else
	{
		if (f->dup_cnt == 0)
		{
			curr->fd_idx = fd;
			file_close(f);
		}
		else
			f->dup_cnt--;
	}

	thread_current()->fdt[fd] = NULL;
}

int 
dup2(int oldfd, int newfd)
{
	if (oldfd == newfd)
		return newfd;

	struct thread *curr = thread_current();
	struct file *f = process_get_file(oldfd);
	if (f == NULL)
		return -1;

	if (newfd < 0 || newfd >= FDT_LIMIT)
		return -1;

	if (f == STDIN)
		curr->stdin_cnt++;
	else if (f == STDOUT)
		curr->stdout_cnt++;
	else
		f->dup_cnt++;

	close(newfd);
	curr->fdt[newfd] = f;
	return newfd;
}

void 
*mmap(void *addr, size_t length, int writable, int fd, off_t offset)
{
	struct file *file = process_get_file(fd);

	if (file == NULL || file == STDIN || file == STDOUT || length == 0 || filesize(fd) == 0 || is_kernel_vaddr(addr) || is_kernel_vaddr(addr + length) || pg_ofs(addr))
	{
		return NULL;
	}	

	if (addr == NULL || addr + length == NULL || spt_find_page(&thread_current()->spt, addr) || offset % PGSIZE)
	{
		return false;
	}	

	return do_mmap(addr, length, writable, file, offset);
}

void 
munmap(void *addr)
{
	do_munmap(addr);
}