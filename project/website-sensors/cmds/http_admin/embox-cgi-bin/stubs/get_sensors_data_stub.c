/**
 * @file
 * @brief Stub to simulate light sensor
 *
 * @date 02.02.2021
 * @author Alexander Kalmuk
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {
	printf(
		"HTTP/1.1 200 OK\r\n"
		"Content-Type: text/event-stream\r\n"
		"Cache-Control: no-cache\r\n"
		"Connection: keep-alive\r\n"
		"\r\n"
	);

	while (1) {
		int s[5];

		for (int i = 0; i < 5; i++) {
			s[i] = random() % 100;
		}

		if (0 > printf("data: [%d, %d, %d, %d, %d]\n\n", s[0], s[1], s[2], s[3], s[4])) {
			break;
		}

		sleep(1);
	}

	return 0;
}
