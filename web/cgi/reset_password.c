#include <stdio.h>

/*
 * largest power of two greater than the sum of the lengths of "username" (8),
 * "=" and the maximum length of an email address (254)
 */
#define BUFSZ   512

#define strcmp4(s, c0, c1, c2, c3)                                            \
    s[0] == c0 && s[1] == c1 && s[2] == c2 && s[3] == c3

int
main(void)
{
    int rv;

    char encoded_buf[BUFSZ];
    char decoded_buf[BUFSZ];

    /*
     * form data
     *
     * this points to a location in the decoded buffer to avoid a string copy
     * operaton and conserve memory
     */
    char *username;

    size_t content_length_n;

    /* environment variables */
    const char *method;
    const char *content_length;

    method = getenv("REQUEST_METHOD");
    if (method == NULL)
        return 1;

    /*
     * Environment variable strings are null-terminated so short-circuiting of
     * the logical AND (&&) will prevent out-of-bounds on the array.  However,
     * note that this a false positive if "POST" is a prefix of method.
     */
    if (strcmp4(method, 'P', 'O', 'S', 'T')) {
        content_length = getenv("CONTENT_LENGTH");
        if (content_length == NULL)
            return 1;

        /* TODO: replace deprecated atoi() */
        content_length_n = atoi(content_length);
        if (content_length_n > (sizeof encoded_buf) - 1)
            return 1;

        read(STDIN_FILENO, encoded_buf, content_length_n);

        encoded_buf[content_length_n] = '\0';

        uri_pct_decode(encoded_buf, decoded_buf);

        rv = cgi_process_content(decoded_buf);
        if (rv == -1)
            return 1;
    }

    return 0;
}
