src = open(r'K:\deepseek_harness_workplace\stm32project\STM32_2_motor\Core\Src\uart_debug.c',
           encoding='utf-8').read()

start = src.index('static void UART_Debug_Help(void)')
end = src.index('\n}', start)
body = src[start:end]

ESC = {'r': '\r', 'n': '\n', 't': '\t', '"': '"', '\\': '\\'}
lits = []
i = 0
while i < len(body):
    if body[i] == '"':
        i += 1
        buf = []
        while i < len(body) and body[i] != '"':
            if body[i] == '\\':
                buf.append(ESC.get(body[i + 1], body[i + 1]))
                i += 2
            else:
                buf.append(body[i])
                i += 1
        lits.append(''.join(buf))
    i += 1

chars = sum(len(l) for l in lits)
bytez = sum(len(l.encode('utf-8')) for l in lits)

print('HELP characters        :', chars)
print('HELP UTF-8 bytes       :', bytez, '  <-- this is what the TX ring must hold')
print('banner(25) + HELP      :', 25 + bytez)
print()
for size in (1024, 2048, 3072):
    print('TX_BUF_SIZE %5d -> usable %5d  fits boot+HELP? %s   margin %d'
          % (size, size - 1, (25 + bytez) <= (size - 1), (size - 1) - (25 + bytez)))
