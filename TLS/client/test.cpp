#include <stdio.h>
#include <ncurses.h>
WINDOW* wintest = newwin(20, 20, 0, 0);

int main()
{

	
	initscr();
	cbreak();
	noecho();
	WINDOW* wintest = newwin(20, 20, 0, 0);
	char c;
	wprintw(wintest, "abc\n");
	wprintw(wintest, "abc\n");

	//wprintw(stdscr, "abc");

	c = getch();
	wrefresh(wintest);

	c = getch();
	endwin();
	return 0;
}
