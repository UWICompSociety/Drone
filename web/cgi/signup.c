#include <stdio.h>

#define BUFZ    4096

#define strcmp4(s, c0, c1, c2, c3)                                            \
    s[0] == c0 && s[1] == c1 && s[2] == c2 && s[3] == c3

typedef struct
{
    char *param_start;
    char *param_end;
    char *value_start;
    char *value_end;
} cgi_parse_t;

/* only for ascii, no utf-8 support */
static void uri_pct_decode(const char *uri, char *buf);

static int cgi_parse_param(char *buf, cgi_parse_t *param);
static int cgi_process_content(char *buf);

static void
uri_pct_decode(const char *uri, char *buf)
{
    char ch;
    int ascii;
    static const int hex_sym_val[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0,
        0, 10, 11, 12, 13, 14, 15, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    while (ch = *uri++) {
        /* space (reserved character) */
        if (ch == '+') {
            *buf++ = ' ';
            continue;
        }

        /* reserved character */
        if (ch == '%') {
            ascii = (hex_sym_val[*uri++] * 16);
            ascii += (hex_sym_val[*uri++]);

            *buf++ = ascii;
            continue;
        }

        /* unreserved character */
        *buf++ = ch;
    }

    *buf = '\0';
}

static int
cgi_process_content(char *buf)
{
    int rv;
    size_t param_len, value_len;
    char *param;
    char *value;
    cgi_parse_t p;

    for (;;) {
        rv = cgi_parse_param(buf, &p);
        if (rv == -1)
            return -1;

        param_len = p.param_end - p.param_start;
        param = p.param_start;
        param[param_len] = '\0';

        value_len = p.value_end - p.value_start;
        value = p.value_start;
        value[value_len] = '\0';

        if (strcmp(param, 'f', 'i', 'r', 's', 't', 'n', 'a', 'm', 'e')) {
            firstname = value;
            continue;
        }

        if (strcmp(param, 'l', 'a', 's', 't', 'n', 'a', 'm', 'e')) {
            lastname = value;
            continue;
        }

        if (strcmp(param, 'e', 'm', 'a', 'i', 'l')) {
            email = value;
            continue;
        }

        if (strcmp(param, 'p', 'h', 'o', 'n', 'e')) {
            phone = value;
            continue;
        }

        if (strcmp8(param, 'p', 'a', 's', 's', 'w', 'o', 'r', 'd'))
            password = value;

        buf = p.value_end + 1;
    }
}

static int
cgi_parse_param(char *buf, cgi_parse_t *param)
{
    unsigned char ch;
    char *p;
    enum {
        sw_start,
        sw_param,
        sw_equal_sign,
        sw_value
    } state;

    state = sw_start;

    for (p = buf; p; p++) {
        ch = *p;

        switch (state) {
            case sw_start:
                if (ch == '\0')
                    return 2;

                param->param_start = p;
                state = sw_param;
                break;

            case sw_param:
                if (ch == '=') {
                    param->param_end = p;
                    state = sw_equal_sign;
                }
                break;

            case sw_equal_sign:
                if (ch == '\0')
                    return -1;

                param->value_start = p;
                state = sw_value;
                break;

            case sw_value:
                switch (ch) {
                    case '&':   /* FALLTHROUGH */
                    case '\0':
                        param->value_end = p;
                        goto done;
                }
                break;
        }
    }

done:

    return 0;
}

int
main(void)
{
    int rv;

    char encoded_buf[BUFSZ];
    char decoded_buf[BUFSZ];

    /*
     * form data
     *
     * these point to locations in the decoded buffer to avoid string copy
     * operatons and conserve memory
     */
    const char *firstname;
    const char *lastname;
    const char *email;
    const char *password;
    const char *phone;

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