/*
    Copyright 2019, 2021, 2022 Joel Svensson  svenssonjoel@yahoo.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <ctype.h>
#include <lbm_types.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <stdbool.h>

#include "compression.h"
#include "tokpar.h"

#define  KEY  0
#define  CODE 1

/* The codes are generated using python script in utils directory
   - Depends on the Huffman library (pip3 install huffman)
   - exec(open('gen_codes.py').read())
   - print(make_c())
*/

#define NUM_CODES 72
#define MAX_KEY_LENGTH 6
#define MAX_CODE_LENGTH 11
char *codes[NUM_CODES][2] = {
    { "nil", "11110101011" },
    { "cdr", "11110101100" },
    { "car", "11110101101" },
    { "cons", "11110101110" },
    { "let", "11110101111" },
    { "define", "1111010000" },
    { "progn", "1111010001" },
    { "quote", "1111010010" },
    { "list", "1111010011" },
    { "if", "1111010100" },
    { "lambda", "11110101010" },
    { "]", "1100" },
    { "[", "1001" },
    { "((", "1110" },
    { "))", "1101" },
    { ")", "1010" },
    { "(", "1011" },
    { "?", "001110" },
    { "!", "001010" },
    { "z", "010100" },
    { "y", "010111" },
    { "x", "011110" },
    { "w", "011000" },
    { "v", "100010" },
    { "u", "001001" },
    { "t", "000110" },
    { "s", "010000" },
    { "r", "010001" },
    { "q", "001011" },
    { "p", "010101" },
    { "o", "010010" },
    { "n", "000100" },
    { "m", "100011" },
    { "l", "001000" },
    { "k", "001101" },
    { "j", "011010" },
    { "i", "001111" },
    { "h", "001100" },
    { "g", "111100" },
    { "f", "011100" },
    { "e", "011101" },
    { "d", "100001" },
    { "c", "010011" },
    { "b", "011001" },
    { "a", "100000" },
    { "9", "0001110" },
    { "8", "0001111" },
    { "7", "0001010" },
    { "6", "0001011" },
    { "5", "0101100" },
    { "4", "0101101" },
    { "3", "0110110" },
    { "2", "0110111" },
    { "1", "0111110" },
    { "0", "0111111" },
    { "_", "11111100" },
    { ",@", "0000101" },
    { ",", "11110110" },
    { "`", "0000000" },
    { " ", "0000001" },
    { "'", "11111001" },
    { "\\", "0000110" },
    { "\"", "11111101" },
    { "#", "11111111" },
    { ".", "0000100" },
    { ">", "11110111" },
    { "<", "11111000" },
    { "=", "11111010" },
    { "/", "11111011" },
    { "*", "0000010" },
    { "-", "0000111" },
    { "+", "0000011" }
    };

int match_longest_key(char *string) {

  int longest_match_ix = -1;
  unsigned int longest_match_length = 0;
  unsigned int n = (unsigned int)strlen(string);

  for (int i = 0; i < NUM_CODES; i ++) {
    unsigned int s_len = (unsigned int)strlen(codes[i][KEY]);
    if (s_len <= n) {
      if (strncasecmp(codes[i][KEY], string, s_len) == 0) {
        if (s_len > longest_match_length) {
          longest_match_ix = i;
          longest_match_length = s_len;
        }
      }
    }
  }
  return longest_match_ix;
}

int match_longest_code(char *string, uint32_t start_bit, uint32_t total_bits) {

  uint32_t bits_left = total_bits - start_bit;
  int longest_match_ix = -1;
  unsigned int longest_match_length = 0;

  for (int i = 0; i < NUM_CODES; i++) {
    unsigned int s_len = (unsigned int)strlen(codes[i][CODE]);
    if ((uint32_t)s_len <= bits_left) {
      bool match = true;
      for (uint32_t b = 0; b < (uint32_t)s_len; b ++) {
        uint32_t byte_ix = (start_bit + b) / 8;
        uint32_t bit_ix  = (start_bit + b) % 8;

        char *code_str = codes[i][CODE];

        if (((string[byte_ix] & (1 << bit_ix)) ? '1' : '0') !=
              code_str[b]) {
          match = false;
        }
      }
      if (match && (s_len > longest_match_length)) {
        longest_match_length = s_len;
        longest_match_ix = i;
      }
    }
  }
  return longest_match_ix;
}

int compressed_length(char *string) {
  uint32_t i = 0;

  uint32_t n = (uint32_t)strlen(string);
  int comp_len = 0; // in bits

  bool string_mode = false;
  bool gobbling_whitespace = false;

  while (i < n) {
    if (string_mode) {
      if (string[i] == '\"'  &&
          !(string[i-1] == '\\')) {
        string_mode = false;
        comp_len += 8;
        i++;
      } else {
        comp_len += 8;
        i++;
      }

    } else {

      // Gobble up any comments
      if (string[i] == ';' ) {
        while (string[i] && string[i] != '\n') {
          i++;
        }
        continue;
      }

      if ( string[i] == '\n' ||
           string[i] == ' '  ||
           string[i] == '\t' ||
           string[i] == '\r') {
        gobbling_whitespace = true;
        i ++;
        continue;
      } else if (gobbling_whitespace) {
        gobbling_whitespace = false;
        i--;
      }

      if (string[i] == '\"') string_mode = true;

      int ix;
      if (isspace(string[i])) {
        ix = match_longest_key(" ");
      } else {
        ix = match_longest_key(string + i);
      }

      if (ix == -1)return -1;
      unsigned int code_len = (unsigned int)strlen(codes[ix][1]);
      comp_len += (int)code_len;
      i += (unsigned int)strlen(codes[ix][0]);
    }
  }
  return comp_len;
}

void set_bit(char *c, int bit_pos, bool set) {
  char bval = 0;
  if (bit_pos <= 7) {
    bval = (char)(1 << bit_pos);
  }
  if (set) {
    *c = (char)(*c | bval);
  } else {
    *c = (char)(*c & ~bval);
  }
}

void emit_string_char_code(char *compressed, char c, int *bit_pos) {

  for (int i = 0; i < 8; i ++) {
    int byte_ix = (*bit_pos) / 8;
    int bit_ix  = (*bit_pos) % 8;
    bool s = (c & (1 << i));
    set_bit(&compressed[byte_ix], bit_ix, s);
    *bit_pos = *bit_pos + 1;
  }
}

void emit_code(char *compressed, char *code, int *bit_pos) {
  unsigned int n = (unsigned int)strlen(code);

  for (unsigned int i = 0; i < n; i ++) {
    int byte_ix = (*bit_pos) / 8;
    int bit_ix  = (*bit_pos) % 8;
    bool s = (code[i] == '1');
    set_bit(&compressed[byte_ix], bit_ix, s);
    *bit_pos = *bit_pos + 1;
  }
}

void emit_key(char *dest, char *key, int nk, uint32_t *char_pos) {

  for (int i = 0; i < nk; i ++) {
    dest[*char_pos] = key[i];
    *char_pos = *char_pos + 1;
  }
}

char read_character(char *src, uint32_t *bit_pos) {

  char c = 0;

  for (int i = 0; i < 8; i ++) {
    uint32_t byte_ix = (*bit_pos)/8;
    uint32_t bit_ix  = (*bit_pos)%8;
    bool s = src[byte_ix] & (1 << bit_ix);
    set_bit(&c, i, s);
    *bit_pos = *bit_pos + 1;
  }
  return c;
}

char *lbm_compress(char *string, uint32_t *res_size) {

  int c_size_bits_i = compressed_length(string);
  uint32_t c_size_bits = 0;
  if ( c_size_bits_i >= 0 ) {
    c_size_bits = (uint32_t)compressed_length(string);
  }

  uint32_t c_size_bytes = 4 + (c_size_bits/8);
  if (c_size_bits % 8 > 0) {
    c_size_bytes += 1;
  }

  uint32_t header_value = c_size_bits;

  if (header_value == 0) return NULL;

  char *compressed = malloc(c_size_bytes);
  if (!compressed) return NULL;
  memset(compressed, 0, c_size_bytes);
  *res_size = c_size_bytes;
  int bit_pos = 0;

  compressed[0] = (char)header_value;
  compressed[1] = (char)(header_value >> 8);
  compressed[2] = (char)(header_value >> 16);
  compressed[3] = (char)(header_value >> 24);
  bit_pos = 32;

  bool string_mode = false;
  bool gobbling_whitespace = false;
  uint32_t n = (uint32_t) strlen(string);
  uint32_t i = 0;

  while (i < n) {
    if (string_mode) {

      if (string[i] == '\"' &&
          !(string[i-1] == '\\')) {
        emit_string_char_code(compressed, '\"', &bit_pos);
        i ++;
        string_mode = false;
        continue;
      } else {
        emit_string_char_code(compressed, string[i], &bit_pos);
        i++;
      }

    } else {

      // Gobble up any comments
      if (string[i] == ';' ) {
        while (string[i] && string[i] != '\n') {
          i++;
        }
        continue;
      }

      // gobble up whitespaces
      if ( string[i] == '\n' ||
           string[i] == ' '  ||
           string[i] == '\t' ||
           string[i] == '\r') {
        gobbling_whitespace = true;
        *(string + i) = ' ';
        i ++;
        continue;
      } else if (gobbling_whitespace) {
        gobbling_whitespace = false;
        i--;
      }

      /* Compress string-starting " character */
      if (string[i] == '\"') {
        string_mode = true;
      }
      int ix = match_longest_key(&string[i]);

      if (ix == -1) {
        free(compressed);
        return NULL;
      }

      emit_code(compressed, codes[ix][CODE], &bit_pos);

      i += (unsigned int)strlen(codes[ix][0]);
    }
  }

  return compressed;
}


void lbm_init_compression_state(decomp_state *s, char *src) {
  memcpy(&s->compressed_bits, src, 4);
  s->i = 32;
  s->string_mode = false;
  s->last_string_char = 0;
  s->src = src;
}

int lbm_decompress_incremental(decomp_state *s, char *dest_buff, uint32_t dest_n) {

  memset(dest_buff, 0, dest_n);
  uint32_t char_pos = 0;

  if (s->i < s->compressed_bits + 32) {
     if (s->string_mode) {
      char c = read_character(s->src, &s->i);
      if (c == '\"') {
        if (s->last_string_char != '\\') {
          s->string_mode = false;
          s->last_string_char = 0;
        }
      }
      s->last_string_char = c;
      dest_buff[0] = c;
      return 1;
    }

    int ix = match_longest_code(s->src, s->i, (s->compressed_bits + 32));
    if (ix == -1) {
      return -1;
    }

    if( strlen(codes[ix][KEY]) == 1 &&
        strncmp(codes[ix][KEY], "\"", 1) == 0) {
      s->string_mode = true;
      s->last_string_char = 0;
    }

    unsigned int n_bits_decoded = (unsigned int)strlen(codes[ix][CODE]);
    emit_key(dest_buff, codes[ix][KEY], (int)strlen(codes[ix][KEY]), &char_pos);
    s->i+=n_bits_decoded;
    return (int)char_pos;

  } else {
    return 0;
  }

}

bool lbm_decompress(char *dest, uint32_t dest_n, char *src) {

  uint32_t char_pos = 0;

  char dest_buff[32];
  int num_chars = 0;
  decomp_state s;

  memset(dest, 0, dest_n);

  lbm_init_compression_state(&s, src);

  while (true) {

    num_chars = lbm_decompress_incremental(&s, dest_buff, 32);
    if (num_chars == 0) break;
    if (num_chars == -1) return false;

    for (int i = 0; i < num_chars; i ++) {
      dest[char_pos++] = dest_buff[i];
    }
  }
  return true;
}

/* Implementation of the parsing interface */
bool more_compressed(lbm_tokenizer_char_stream_t *str) {
  tokenizer_compressed_state_t *s = (tokenizer_compressed_state_t*)str->state;
  bool more =
    (s->ds.i < s->ds.compressed_bits + 32) ||
    (s->buff_pos < s->decomp_bytes);
  return more;
}

char get_compressed(lbm_tokenizer_char_stream_t *str) {
  tokenizer_compressed_state_t *s = (tokenizer_compressed_state_t*)str->state;

  if (s->ds.i >= s->ds.compressed_bits + 32 &&
      (s->buff_pos >= s->decomp_bytes)) {
    return 0;
  }

  if (s->buff_pos >= s->decomp_bytes) {
    int n = lbm_decompress_incremental(&s->ds, s->decomp_buff,DECOMP_BUFF_SIZE);
    if (n == 0) {
      return 0;
    }
    s->decomp_bytes = n;
    s->buff_pos = 0;
  }
  char c = s->decomp_buff[s->buff_pos];
  s->buff_pos += 1;
  return c;
}

char peek_compressed(lbm_tokenizer_char_stream_t *str, unsigned int n) {
  tokenizer_compressed_state_t *s = (tokenizer_compressed_state_t *)str->state;

  tokenizer_compressed_state_t old;

  memcpy(&old, s, sizeof(tokenizer_compressed_state_t));

  char c = get_compressed(str);;
  for (unsigned int i = 1; i <= n; i ++) {
    c = get_compressed(str);
  }

  memcpy(str->state, &old, sizeof(tokenizer_compressed_state_t));
  return c;
}

void drop_compressed(lbm_tokenizer_char_stream_t *str, unsigned int n) {
  for (unsigned int i = 0; i < n; i ++) {
    get_compressed(str);
  }
}

unsigned int row_compressed(lbm_tokenizer_char_stream_t *str) {
  (void) str;
  return 0;
}

unsigned int column_compressed(lbm_tokenizer_char_stream_t *str) {
  (void) str;
  return 0;
}

void lbm_create_char_stream_from_compressed(tokenizer_compressed_state_t *ts,
                                                    lbm_tokenizer_char_stream_t *str,
                                                    char *bytes) {
  ts->decomp_bytes = 0;
  memset(ts->decomp_buff, 0, 32);
  ts->buff_pos = 0;

  lbm_init_compression_state(&ts->ds, bytes);

  str->state = ts;
  str->more = more_compressed;
  str->get = get_compressed;
  str->peek = peek_compressed;
  str->drop = drop_compressed;
}
