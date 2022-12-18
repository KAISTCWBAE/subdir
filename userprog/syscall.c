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

#define STDIN 1	 // 표준 입력
#define STDOUT 2 // 표준 출력

// readers-writers를 위한 semaphore와 cnt

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

bool chdir(const char *dir);
bool mkdir(const char *dir);
bool readdir(int fd, char *name);
bool isdir(int fd);
int inumber(int fd);
int symlink(const char *target, const char *linkpath);

int dup2(int oldfd, int newfd);

void syscall_init(void)
{
	lock_init(&file_lock);

	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48 |
							((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t)syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			  FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

/*
address의 유효성 검사
1. kernel address인가?
2. NULL 값인가?
3. 할당 받은 VM의 address인가?
유효하지 않으면 thread 종료
*/
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

	if (buffer <= USER_STACK && buffer >= rsp)
		return;

	for (; start_page <= end_page; start_page += PGSIZE)
	{
		struct page *page = check_address(start_page);
		if (page == NULL)
			exit(-1);

		if (to_write == true && page->writable == false)
			exit(-1);
	}
}

/*
The main system call interface
user mode로 돌아갈 때 사용할 if를 syscall_handler의 인자로 넣어줌
*/
void syscall_handler(struct intr_frame *f)
{
	/* Projects 2 and later. */
	// SYS_HALT,		/* Halt the operating system. */
	// SYS_EXIT,		/* Terminate this process. */
	// SYS_FORK,		/* Clone current process. */
	// SYS_EXEC,		/* Switch current process. */
	// SYS_WAIT,		/* Wait for a child process to die. */
	// SYS_CREATE,		/* Create a file. */
	// SYS_REMOVE,		/* Delete a file. */
	// SYS_OPEN,		/* Open a file. */
	// SYS_FILESIZE,	/* Obtain a file's size. */
	// SYS_READ,		/* Read from a file. */
	// SYS_WRITE,		/* Write to a file. */
	// SYS_SEEK,		/* Change position in a file. */
	// SYS_TELL,		/* Report current position in a file. */
	// SYS_CLOSE,		/* Close a file. */

	/* Project 3 and optionally project 4. */
	// SYS_MMAP,		/* Map a file into memory. */
	// SYS_MUNMAP,		/* Remove a memory mapping. */

	/* Project 4 only. */
	// SYS_CHDIR,		/* Change the current directory. */
	// SYS_MKDIR,		/* Create a directory. */
	// SYS_READDIR,		/* Reads a directory entry. */
	// SYS_ISDIR,		/* Tests if a fd represents a directory. */
	// SYS_INUMBER,		/* Returns the inode number for a fd. */
	// SYS_SYMLINK,

	/* Extra for Project 2 */
	// SYS_DUP2			/* Duplicate the file descriptor */

	thread_current()->user_rsp = f->rsp;

	switch (f->R.rax)
	{
	case SYS_HALT:
		halt();
		break;

	case SYS_EXIT:
		// argv[0]: int status
		exit(f->R.rdi);
		break;

	case SYS_FORK:
		// argv[0]: const char *thread_name
		check_address(f->R.rdi);

		f->R.rax = fork(f->R.rdi, f);
		break;

	case SYS_EXEC:
		// argv[0]: const char *file
		check_address(f->R.rdi);

		if (exec(f->R.rdi) < 0)
			exit(-1);
		break;

	case SYS_WAIT:
		// argv[0]: tid_t pid
		f->R.rax = wait(f->R.rdi);
		break;

	case SYS_CREATE:
		// argv[0]: const char *file
		// argv[1]: unsigned initial_size
		check_address(f->R.rdi);

		f->R.rax = create(f->R.rdi, f->R.rsi);
		break;

	case SYS_REMOVE:
		// argv[0]: const char *file
		check_address(f->R.rdi);

		f->R.rax = remove(f->R.rdi);
		break;

	case SYS_OPEN:
		// argv[0]: const char *file
		check_address(f->R.rdi);

		f->R.rax = open(f->R.rdi);
		break;

	case SYS_FILESIZE:
		// argv[0]: int fd
		f->R.rax = filesize(f->R.rdi);
		break;

	case SYS_READ:
		// argv[0]: int fd
		// argv[1]: void *buffer
		// argv[2]: unsigned size
		check_valid_buffer(f->R.rsi, f->R.rdx, f->rsp, 1);

		f->R.rax = read(f->R.rdi, f->R.rsi, f->R.rdx);
		break;

	case SYS_WRITE:
		// argv[0]: int fd
		// argv[1]: const void *buffer
		// argv[2]: unsigned size
		check_valid_buffer(f->R.rsi, f->R.rdx, f->rsp, 0);

		f->R.rax = write(f->R.rdi, f->R.rsi, f->R.rdx);
		break;

	case SYS_SEEK:
		// argv[0]: int fd
		// argv[1]: unsigned position
		seek(f->R.rdi, f->R.rsi);
		break;

	case SYS_TELL:
		// argv[0]: int fd
		f->R.rax = tell(f->R.rdi);
		break;

	case SYS_CLOSE:
		// argv[0]: int fd
		close(f->R.rdi);
		break;

	case SYS_MMAP:
		// argv[0]: void *addr
		// argv[1]: size_t length
		// argv[2]: int writable
		// argv[3]: int fd
		// argv[4]: off_t offset
		f->R.rax = mmap(f->R.rdi, f->R.rsi, f->R.rdx, f->R.r10, f->R.r8);
		break;

	case SYS_MUNMAP:
		// argv[0]: void *addr
		check_address(f->R.rdi);

		munmap(f->R.rdi);
		break;

	case SYS_CHDIR:
		// argv[0]: const char *dir
		check_address(f->R.rdi);

		f->R.rax = chdir(f->R.rdi);
		break;

	case SYS_MKDIR:
		// argv[0]: const char *dir
		check_address(f->R.rdi);

		f->R.rax = mkdir(f->R.rdi);
		break;

	case SYS_READDIR:
		// argv[0]: int fd
		// argv[1]: char *name
		check_address(f->R.rsi);

		f->R.rax = readdir(f->R.rdi, f->R.rsi);
		break;

	case SYS_ISDIR:
		// argv[0]: int fd
		f->R.rax = isdir(f->R.rdi);
		break;

	case SYS_INUMBER:
		// argv[0]: int fd
		f->R.rax = inumber(f->R.rdi);
		break;

	case SYS_SYMLINK:
		// argv[0]: const char *target
		// argv[0]: const char *linkpath
		check_address(f->R.rdi);
		check_address(f->R.rsi);

		f->R.rax = symlink(f->R.rdi, f->R.rsi);
		break;

	case SYS_DUP2:
		// argv[0]: int oldfd
		// argv[1]: int newfd
		f->R.rax = dup2(f->R.rdi, f->R.rsi);
		break;
	}
}

/* pintOS 종료 */
void halt(void)
{
	power_off();
}

/* 현재 process 종료 */
void exit(int status)
{
	thread_current()->exit_status = status;
	printf("%s: exit(%d)\n", thread_name(), thread_current()->exit_status);

	thread_exit();
}

/*
자신과 같은 file을 실행하는 자식 process를 만든다
자식 process는 자신이 fork system call을 호출한 이후 부터 실행된다
syscall_handler가 인자로 받은 현재 thread의 user mode if를 인자로 받는다
*/
tid_t fork(const char *thread_name, struct intr_frame *if_)
{
	return process_fork(thread_name, if_);
}

/*
현재 process가 입력 받은 file을 실행하도록 바꾼다
thread name은 바뀌지 않는다
*/
int exec(const char *file)
{
	/* Make a copy of FILE_NAME.
	 * Otherwise there's a race between the caller and load(). */
	char *fn_copy = palloc_get_page(PAL_ZERO);
	if (fn_copy == NULL)
		return -1;

	strlcpy(fn_copy, file, PGSIZE);

	if (process_exec(fn_copy) < 0)
		return -1;

	return 0;
}

/* 입력 받은 pid를 가진 자식 process가 종료될 때까지 sleep */
int wait(tid_t pid)
{
	return process_wait(pid);
}

/*
initial_size를 가진 file 생성
만들지만 open하지는 않는다
*/
bool create(const char *file, unsigned initial_size)
{
	lock_acquire(&file_lock);
	bool succ = filesys_create(file, initial_size);
	lock_release(&file_lock);

	return succ;
}

/* 해당 file 삭제 */
bool remove(const char *file)
{
	return filesys_remove(file);
}

/* 입력 받은 file을 열어서 file descripter 생성 */
int open(const char *file)
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

int filesize(int fd)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	return file_length(f);
}

/* fd를 size만큼 buffer에 읽어온다 */
int read(int fd, void *buffer, unsigned size)
{
	struct file *f = process_get_file(fd);
	if (f == NULL || f == STDOUT)
		return -1;

	int read_result;

	/*
	표준 입력
	f는 pointer이지만 1, 2 값일 때 STD_IN, STD_OUT으로 사용한다
	1, 2는 유효하지 않은 address지만 address로 접근하지 않고 int처럼 활용
	input_getc()는 한 글자씩 입력 받는 함수
	*/
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

/* buffer에서 size만큼 fd에 쓴다 */
int write(int fd, const void *buffer, unsigned size)
{
	struct file *f = process_get_file(fd);
	if (f == NULL || f == STDIN)
		return -1;

	int write_result;

	// 표준 출력
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

/* fd의 pos를 인자로 받은 pos로 바꾼다 */
void seek(int fd, unsigned pos)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	file_seek(f, pos);
}

/* fd의 pos를 return */
unsigned tell(int fd)
{
	if (fd < 2)
		return -1;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return -1;

	return file_tell(f);
}

/*
fd를 닫고 fdt에서 fd를 삭제한다

dup2의 경우 같은 file pointer를 여러 fd가 가지고 있을 수 있다
한 fd를 close한다고 해서 file_close를 해버리면 다른 fd들이 쓰레기값을 가지게 된다
그래서 struct file에 자신이 얼마나 복사됬는지를 기록해 두는 dup_cnt를 추가해서 관리
만약 복사되어 있다면 dup_cnt만 줄이고 fd를 삭제한다
복사되어 있지 않다면(dup_cnt==0) file_close를 한다

stdin_cnt와 stdout_cnt는 별 쓸모 없는 것 같음
*/
void close(int fd)
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

/*
oldfd가 가리키는 struct file pointer를 newfd도 가리키게 한다
STD_IN, STD_OUT의 경우도 복사 가능
*/
int dup2(int oldfd, int newfd)
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

/**************** project 3: virtual memory *******************/
void *mmap(void *addr, size_t length, int writable, int fd, off_t offset)
{
	struct file *file = process_get_file(fd);
	if (file == NULL || file == STDIN || file == STDOUT)
		return NULL;

	if (length == 0 || filesize(fd) == 0)
		return NULL;

	if (is_kernel_vaddr(addr) || is_kernel_vaddr(addr + length) || pg_ofs(addr))
		return NULL;

	if (addr == NULL || addr + length == NULL)
		return false;

	if (spt_find_page(&thread_current()->spt, addr) || offset % PGSIZE)
		return false;

	return do_mmap(addr, length, writable, file, offset);
}

void munmap(void *addr)
{
	do_munmap(addr);
}

bool chdir(const char *name)
{
	struct thread *curr = thread_current();

	char *copy_name = (char *)malloc(strlen(name) + 1);
	if (copy_name == NULL)
		return false;

	strlcpy(copy_name, name, strlen(name) + 1);

	if (strlen(copy_name) == 0)
		return false;

	struct dir *dir;

	if (copy_name[0] == '/')
		dir = dir_open_root();
	else
		dir = dir_reopen(curr->curr_dir);

	char *token, *save_ptr;
	token = strtok_r(copy_name, "/", &save_ptr);

	struct inode *inode = NULL;
	while (token != NULL)
	{
		if (!dir_lookup(dir, token, &inode))
			goto fail;

		if (!inode_is_dir(inode))
			goto fail;

		dir_close(dir);
		dir = dir_open(inode);

		token = strtok_r(NULL, "/", &save_ptr);
	}

	dir_close(curr->curr_dir);
	curr->curr_dir = dir;
	free(copy_name);

	return true;

fail:
	dir_close(dir);
	if (inode)
		inode_close(inode);

	return false;
}

bool mkdir(const char *dir)
{
	char *copy_dir = (char *)malloc(strlen(dir) + 1);
	if (copy_dir == NULL)
		return false;

	strlcpy(copy_dir, dir, strlen(dir) + 1);

	lock_acquire(&file_lock);
	bool succ = filesys_create_dir(copy_dir);
	lock_release(&file_lock);

	free(copy_dir);

	return succ;
}

bool readdir(int fd, char *name)
{
	if (name == NULL)
		return false;

	struct file *f = process_get_file(fd);
	if (f == NULL)
		return false;

	if (inode_is_dir(f->inode) == INODE_FILE)
		return false;

	struct dir *dir = f;

	bool succ = dir_readdir(dir, name);

	return succ;
}

bool isdir(int fd)
{
	struct file *f = process_get_file(fd);
	if (f == NULL)
		return false;

	return inode_is_dir(f->inode);
}

int inumber(int fd)
{
	struct file *f = process_get_file(fd);
	if (f == NULL)
		return false;

	return inode_get_inumber(f->inode);
}

int symlink(const char *target, const char *linkpath)
{
	char *copy_linkpath = (char *)malloc(strlen(linkpath) + 1);
	strlcpy(copy_linkpath, linkpath, strlen(linkpath) + 1);

	lock_acquire(&file_lock);
	int result = filesys_create_link(target, copy_linkpath);
	lock_release(&file_lock);

	free(copy_linkpath);

	return result;
}