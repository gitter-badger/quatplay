#ifndef __CONFIG_READER_H__
#define __CONFIG_READER_H__

typedef void (*LineReaderCallback)( const char *line, void *user );
typedef LineReaderCallback (*HeaderReaderCallback)( const char *line, void *user );

// reads a file with square bracket header sections.
// chomps the newlines from all lines sent to callbacks.
// Trims the brackets from the header lines before calling the HeaderReaderCallback
int OpenConfigAndCallbackPerLine(
		const char *filename,
		HeaderReaderCallback hcb,
		LineReaderCallback lcb,
		void *user
		);

#endif
