#include "ConfigReader.h"

// for fopen, fclose, fgets
#include <stdio.h>

// for strlen
#include <string.h>

// for whitespace
#include <wctype.h>

inline void chomp( char *str ) {
	char * lastchar = str + strlen(str) - 1;
	while( lastchar >= str && iswspace( *lastchar ) ) {
		*lastchar = 0;
		--lastchar;
	}
}

int OpenConfigAndCallbackPerLine( const char *source, HeaderReaderCallback hcb, LineReaderCallback lcb, void *user ) {
	LineReaderCallback cb = lcb;
	if( FILE *fp = fopen( source, "r" ) ) {
		//logf( 3, "Loaded input\n" );
		char buffer[1024], *current;
		while( ( current = fgets( buffer, sizeof( buffer )-1, fp ) ) ) {
			chomp( current );
			size_t l = strlen( current );
			if( l && current[0] != '#' ) {
				if( current[0] == '[' && current[l-1] == ']' ) {
					current[l-1] = 0;
					//logf( 3, "Calling met a header\n" );
					cb = hcb( current+1, user );
				} else if( cb ) {
					//logf( 3, "Calling callback\n" );
					cb( current, user );
				}
			}
		}
		fclose( fp );
		return 0;
	}
	return 404;
}

