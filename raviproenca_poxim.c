#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define CR 26
#define IPC 27
#define IR 28
#define PC 29
#define SP 30
#define SR 31

#define CY 0
#define IE 1
#define IV 2
#define OV 3
#define SN 4 
#define ZD 5
#define ZN 6

// Funções declaradas
void printArguments(int argc, char* argv[]);
void initializeMemory(uint8_t** MEM8, uint32_t* R);
void loadMemoryFromFile(const char* filename, uint8_t* MEM8);
void printMemory(uint8_t* MEM8);
void executeInstructions(uint8_t* MEM8, uint32_t* R, FILE* output);
void handleInstruction(uint8_t* MEM8, uint32_t* R, FILE* output, uint8_t* executa);
void handleInterrupt(uint8_t* MEM8, uint32_t* R, FILE* output);
void updateWatchdog(uint8_t* MEM8, uint32_t* R, FILE* output);
void executarFPU(uint8_t* MEM8, uint32_t* R, FILE* output);
void updateFPU(uint8_t* MEM8, uint32_t* R, FILE* output);

uint32_t floatToInt(float value);
int calcularCiclos(float valueX, float valueY, FILE* output);
uint32_t valorInstrucao = 0;

char* registerNameUpper(int number);
char* registerNameLower(int number);

typedef struct {
    uint32_t EN;       // Bit de habilitação do watchdog
    uint32_t COUNTER; // Contador do watchdog
} Watchdog;

// Inicializando o watchdog
Watchdog wd = {0, 0};

typedef struct {
    uint8_t OUT; // Campo OUT para escrita e leitura (8 bits)
    char buffer[15000]; // Buffer para armazenar os caracteres escritos
    int bufferIndex;   // Índice atual no buffer
} Terminal;

// Inicializando o terminal
Terminal terminal = {0, {0}, 0};

typedef struct {
    uint32_t X;  // Registrador X (RW)
    uint32_t Y;  // Registrador Y (RW)
    uint32_t Z;  // Registrador Z (RW)
    uint8_t ST;  // Status (RO): 0 = Pronto, 1 = Erro
    uint8_t OP;  // Código da Operação (RW)
    int ciclos2;
    int ciclos3;
    int ciclos4;
    int interrupcao;
    int fpuAtivado;
    int fpuContador;
} FPU;

// Inicializando a FPU
FPU fpu = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct {
    uint32_t codigoInterrupcao; // Código da interrupção
    uint32_t prioridade;        // Nível de prioridade (0 = maior prioridade)
} Interrupcao;

Interrupcao interrupcaoPendente;
int interrupcaoAtiva = 0; // Indica se há uma interrupção ativa

typedef struct {
    uint32_t tag[8][2];       // Identificador do bloco para 8 conjuntos e 2 vias (associatividade de 2 vias)
    uint32_t data[8][2][4];   // Dados de 4 palavras para 8 conjuntos e 2 vias
    uint8_t lru[8][2];        // Controle de idade para LRU para cada conjunto e via
    uint8_t valid[8][2];         // Validade de cada bloco (conjunto e via)
    int typeMEML;
    int typeMEMS;
    int cache_hits;
    int cache_accesses;
    float D_hit_rate;
    float I_hit_rate;
} Cache;

// Instâncias para cache de dados e instruções
Cache cacheD, cacheI = {0};

// Função main
int main(int argc, char* argv[]) {
    // Imprime os argumentos
    printArguments(argc, argv);
    
    // Inicializa a memória e os registradores
    uint32_t R[32] = { 0 };
    uint8_t* MEM8 = NULL;
    initializeMemory(&MEM8, R);

    char* registerNameUpper(int number);
    char* registerNameLower(int number);

    // Carrega a memória a partir do arquivo
    FILE* input = fopen(argv[1], "r");
    if (input == NULL) {
        perror("Erro ao abrir o arquivo de entrada");
        return 1;
    }
    loadMemoryFromFile(argv[1], MEM8);
    fclose(input);

    // Imprime o conteúdo da memória
    printMemory(MEM8);

    // Prepara para execução das instruções
    FILE* output = fopen(argv[2], "w");
    if (output == NULL) {
        perror("Erro ao abrir o arquivo de saída");
        free(MEM8);
        return 1;
    }
    fprintf(output, "[START OF SIMULATION]\n");
    
    // Executa as instruções
    executeInstructions(MEM8, R, output);
    fprintf(output, "[END OF SIMULATION]");
    fclose(output);

    // Libera memória
    free(MEM8);

    return 0;
}

// Função para imprimir argumentos
void printArguments(int argc, char* argv[]) {
    printf("Quantidade de argumentos (argc): %i\n", argc);
    for (int i = 0; i < argc; i++) {
        printf("Argumento %i (argv[%i]): %s\n", i, i, argv[i]);
    }
}

// Função para inicializar a memória
void initializeMemory(uint8_t** MEM8, uint32_t* R) {
    *MEM8 = (uint8_t*)calloc(32 * 1024, sizeof(uint8_t)); // Corrigindo o tamanho da alocação
    if (*MEM8 == NULL) {
        perror("Erro ao alocar memória");
        exit(1);
    }
}

// Função para carregar memória a partir de um arquivo
void loadMemoryFromFile(const char* filename, uint8_t* MEM8) {
    FILE* input = fopen(filename, "r");
    if (input == NULL) {
        perror("Erro ao abrir o arquivo de entrada");
        exit(1);
    }
    
    int contador = 0;
    unsigned int temp;
    while (fscanf(input, "%x", &temp) != EOF) {
        MEM8[contador] = (uint8_t)((temp >> 24) & 0xFF);
        MEM8[contador + 1] = (uint8_t)((temp >> 16) & 0xFF);
        MEM8[contador + 2] = (uint8_t)((temp >> 8) & 0xFF);
        MEM8[contador + 3] = (uint8_t)(temp & 0xFF);
        contador += 4;
    }
    
    fclose(input);
}

// Função para imprimir o conteúdo da memória
void printMemory(uint8_t* MEM8) {
    printf("\nMEM8:\n");
    for (int i = 0; i < 252; i += 4) {
        printf("0x%08X: 0x%02X 0x%02X 0x%02X 0x%02X\n", i, MEM8[i], MEM8[i + 1], MEM8[i + 2], MEM8[i + 3]);
    }
}

// Função para executar as instruções
void executeInstructions(uint8_t* MEM8, uint32_t* R, FILE* output) {
    uint8_t executa = 1;
    while (executa) {
        handleInstruction(MEM8, R, output, &executa);
        updateWatchdog(MEM8, R, output);
        updateFPU(MEM8, R, output);
        R[PC] = R[PC] + 4;
    }
}

void updateWatchdog(uint8_t* MEM8, uint32_t* R, FILE* output) {
    // Decremento do watchdog
    if (wd.EN == 1) { // Verifica se o bit de habilitação é 1
        if (wd.COUNTER > 0) {
            wd.COUNTER--; // Decrementa o contador
        }
        if (wd.COUNTER == 1) {
            if ((R[SR] & (1 << IE)) != 0) {
                R[IPC] = R[PC] + 4;
            }
        }
        if (wd.COUNTER == 0) {

            // Gera interrupção, se necessário
            if ((R[SR] & (1 << IE)) != 0) { // Verifica se a interrupção está desabilitada
                //inserirInterrupcao(0xE1AC04DA, 1);
                fprintf(output,"[HARDWARE INTERRUPTION 1]\n");
                handleInterrupt(MEM8, R, output);
                R[CR] = 0xE1AC04DA; // Código da interrupção do watchdog
                R[PC] = 0x0000000C;
                wd.EN = 0;
            }
        }
    }
}

uint32_t floatToInt(float value) {
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

int calcularCiclos(float valueX, float valueY, FILE* output) {
    uint32_t x = floatToInt(valueX);
    uint32_t y = floatToInt(valueY);

    int expX = (x >> 23) & 0xFF;  // Extrair o expoente de X
    int expY = (y >> 23) & 0xFF;  // Extrair o expoente de Y

    return abs(expX - expY) + 1;
}


void executarFPU(uint8_t* MEM8, uint32_t* R, FILE* output) {
    float fx, fy, fz;

    // Conversão de Inteiro para Float IEEE 754
    fx = fpu.X;
    fy = fpu.Y;

    // Verificação e Execução de Operação
    switch (fpu.OP) {
        case 0b00000:  // Sem operação
            break;
        case 0b00001:  // Adição Z = X + Y
            fz = fx + fy;
            fpu.Z = fz;

            fpu.ciclos3 = calcularCiclos(fx, fy, output);
            fpu.interrupcao = 3;
            break;
        case 0b00010:  // Subtração Z = X - Y
            fz = fx - fy;
            fpu.Z = fz;

            fpu.ciclos3 = calcularCiclos(fx, fy, output);
            fpu.interrupcao = 3;
            break;
        case 0b00011:  // Multiplicação Z = X * Y
            fz = fx * fy;
            fpu.Z = fz;

            fpu.ciclos3 = calcularCiclos(fx, fy, output);
            fpu.interrupcao = 3;
            break;
        case 0b00100:  // Divisão Z = X / Y
            if (fy == 0) {

                fpu.ciclos2 = calcularCiclos(fx, fy, output);
                fpu.interrupcao = 2;
                break;
            }

            fz = fx / fy;
            fpu.Z = fz;

            fpu.ciclos3 = calcularCiclos(fx, fy, output);
            fpu.interrupcao = 3;
            break;
        case 0b00101:  // Atribuição X = Z
            fpu.X = fpu.Z;

            fpu.ciclos4 = 1;
            fpu.interrupcao = 4;
            break;
        case 0b00110:  // Atribuição Y = Z
            fpu.Y = fpu.Z;

            fpu.ciclos4 = 1;
            fpu.interrupcao = 4;
            break;
        case 0b00111:  // Teto [Z]
            fz = ceil(fx);
            fpu.Z = floatToInt(fz);

            fpu.ciclos4 = 1;
            fpu.interrupcao = 4;
            break;
        case 0b01000:  // Piso [Z]
            fz = floor(fx);
            fpu.Z = floatToInt(fz);

            fpu.ciclos4 = 1;
            fpu.interrupcao = 4;
            break;
        case 0b01001:  // Arredondamento ||Z||
            fz = round(fx);
            fpu.Z = floatToInt(fz);

            fpu.ciclos4 = 1;
            fpu.interrupcao = 4;
            break;
        default:  // Código de operação inválido

            fpu.ciclos2 = 1;
            fpu.interrupcao = 2;
            break;
        }
}

void updateFPU(uint8_t* MEM8, uint32_t* R, FILE* output) {
    if (fpu.fpuAtivado == 1) { // Verifica se o bit de habilitação é 1
        if (fpu.fpuContador > 0) {
            fpu.fpuContador--; // Decrementa o contador
        }
        if (fpu.fpuContador == 1) {
            if ((R[SR] & (1 << IE)) != 0) {
                R[IPC] = R[PC] + 4;
            }
        }
        if (fpu.fpuContador == 0) {

            if (fpu.interrupcao == 2) {
                // Gera interrupção, se necessário
                if ((R[SR] & (1 << IE)) != 0) {  // Verifica se a interrupção está desabilitada
                    //inserirInterrupcao(0x01EEE754, 2);
                    fprintf(output,"[HARDWARE INTERRUPTION 2]\n");
                    handleInterrupt(MEM8, R, output);
                    R[CR] = 0x01EEE754; // Código da interrupção do FPU
                    R[PC] = 0x00000010;
                    fpu.OP = 0;
                    fpu.ST = 1;
                }
            }
            else if (fpu.interrupcao == 3) {
                // Gera interrupção, se necessário
                if ((R[SR] & (1 << IE)) != 0) { // Verifica se a interrupção está desabilitada
                    //inserirInterrupcao(0x01EEE754, 3);
                    fprintf(output,"[HARDWARE INTERRUPTION 3]\n");
                    handleInterrupt(MEM8, R, output);
                    R[CR] = 0x01EEE754; // Código da interrupção do watchdog
                    R[PC] = 0x00000014;
                    fpu.OP = 0;
                    fpu.ST = 0;
                }
            }
            else if (fpu.interrupcao == 4) {
                // Gera interrupção, se necessário
                if ((R[SR] & (1 << IE)) != 0) {  // Verifica se a interrupção está desabilitada
                    //inserirInterrupcao(0x01EEE754, 4);
                    fprintf(output,"[HARDWARE INTERRUPTION 4]\n");
                    handleInterrupt(MEM8, R, output);
                    R[CR] = 0x01EEE754; // Código da interrupção do FPU
                    R[PC] = 0x00000018;
                    fpu.OP = 0;
                    fpu.ST = 0;
                }
            }
            fpu.fpuAtivado = 0; // Desabilita o contador
        }
    }
}

// Função para obter o nome do registrador em maiúsculas
char* registerNameUpper(int number) {
    char* name = (char*)malloc(sizeof(char) * 4);
    switch (number) {
        case IR: return "IR";
        case PC: return "PC";
        case SP: return "SP";
        case SR: return "SR";
        case CR: return "CR";
        case IPC: return "IPC";
        default: sprintf(name, "R%d", number); return name;
    }
}

// Função para obter o nome do registrador em minúsculas
char* registerNameLower(int number) {
    char* name = (char*)malloc(sizeof(char) * 4);
    switch (number) {
        case IR: return "ir";
        case PC: return "pc";
        case SP: return "sp";
        case SR: return "sr";
        case CR: return "cr";
        case IPC: return "ipc";
        default: sprintf(name, "r%d", number); return name;
    }
}


void escreveNaCacheD(uint8_t* MEM8, Cache *cache, uint32_t endereco, uint32_t dado, FILE* output) {
    cacheD.cache_accesses++;
    uint32_t tag = (endereco >> 7);
    uint32_t indice = (endereco >> 4) & 0x07;
    uint32_t offset = (endereco & 0x0F) / 4;
    uint32_t byteOffset = endereco & 0x03;  // Para 8 bits, usamos este offset dentro da palavra
    char instrucao[50] = { 0 };

    int viaLRU = 0, hitVia = -1;

    // Incrementa o LRU de todas as vias válidas da cache
    for (int ind = 0; ind < 8; ind++) {
        for (int via = 0; via < 2; via++) {
            if (cache->valid[ind][via]) {
                if (cache->lru[ind][via] < 255) {
                    cache->lru[ind][via]++;
                }
            }
        }
    }

    // Verificar se há um hit na cache
    for (int i = 0; i < 2; i++) {
        if (cache->valid[indice][i] && cache->tag[indice][i] == tag) {
            hitVia = i;
            break;
        }
        // Guardar a via LRU
        if (cache->lru[indice][i] > cache->lru[indice][viaLRU]) {
            viaLRU = i;
        }
    }

    if (hitVia != -1) {  // Hit na cache
        cacheD.cache_hits++;
        if (cacheD.typeMEMS == 8) {
            // Atualiza o byte dentro da palavra
            uint32_t palavra = cache->data[indice][hitVia][offset];
            palavra = (palavra & ~(0xFF << (8 * (3 - byteOffset)))) | ((dado & 0xFF) << (8 * (3 - byteOffset)));
            cache->data[indice][hitVia][offset] = palavra;

            // Escrita de 8 bits na memória
            MEM8[endereco] = dado & 0xFF;
        } else {
            // Atualiza a palavra inteira na cache
            cache->data[indice][hitVia][offset] = dado;

            // Escrita de 32 bits na memória
            for (int i = 0; i < 4; i++) {
                MEM8[endereco + i] = (dado >> (24 - i * 8)) & 0xFF;
            }
        }

        // Atualiza o LRU da via acessada
        cache->lru[indice][hitVia] = 0;

        // Log da escrita com hit
        sprintf(instrucao, "D_write_hit [%d]->[%d]", indice, hitVia);
        fprintf(output, "0x%08X:\t%-25s\tID=0x%06X,DATA={0x%08X,0x%08X,0x%08X,0x%08X}\n",
                endereco, instrucao, cache->tag[indice][hitVia], cache->data[indice][hitVia][0], 
                cache->data[indice][hitVia][1], cache->data[indice][hitVia][2], cache->data[indice][hitVia][3]);
    } else {  // Miss na cache
        if (cacheD.typeMEMS == 8) {
            // Escrita de 8 bits na memória
            MEM8[endereco] = dado & 0xFF;
        } else {
            // Escrita de 32 bits na memória
            for (int i = 0; i < 4; i++) {
                MEM8[endereco + i] = (dado >> (24 - i * 8)) & 0xFF;
            }
        }

        // Log de miss
        sprintf(instrucao, "D_write_miss [%d]", indice);
        fprintf(output, "0x%08X:\t%-25s\t[0]{VAL=%d,AGE=%d,ID=0x%06X},[1]{VAL=%d,AGE=%d,ID=0x%06X}\n", 
                endereco, instrucao, 
                cache->valid[indice][0], cache->lru[indice][0], cache->tag[indice][0],
                cache->valid[indice][1], cache->lru[indice][1], cache->tag[indice][1]);

        // Como é "no write allocate", a cache não é atualizada
    }
}

uint32_t leDaCacheD(uint8_t* MEM8, Cache *cache, uint32_t endereco, FILE* output) {
    cacheD.cache_accesses++;
    uint32_t tag = (endereco >> 7);
    uint32_t indice = (endereco >> 4) & 0x07;
    uint32_t offset = (endereco & 0x0F) / 4;
    uint32_t byteOffset = endereco & 0x03;  // Para 8 bits
    char instrucao[50] = { 0 };

    int viaLRU = 0, hitVia = -1;

    // Incrementa o LRU de todas as vias válidas da cache
    for (int ind = 0; ind < 8; ind++) {
        for (int via = 0; via < 2; via++) {
            if (cache->valid[ind][via]) {
                if (cache->lru[ind][via] < 255) {
                    cache->lru[ind][via]++;
                }
            }
        }
    }

    // Verificar se há um hit na cache
    for (int i = 0; i < 2; i++) {
        if (cache->valid[indice][i] && cache->tag[indice][i] == tag) {
            hitVia = i;
            break;
        }
        // Guardar a via LRU
        if (cache->lru[indice][i] > cache->lru[indice][viaLRU]) {
            viaLRU = i;
        }
    }

    if (hitVia != -1) {  // Hit na cache
        cacheD.cache_hits++;
        cache->lru[indice][hitVia] = 0;  // Atualiza LRU

        if (cacheD.typeMEML == 8) {
            uint32_t palavra = cache->data[indice][hitVia][offset];
            return (palavra >> (8 * (3 - byteOffset))) & 0xFF;  // Retorna 8 bits
        } else {
            return cache->data[indice][hitVia][offset];  // Retorna 32 bits
        }
    } else {  // Miss na cache
        uint32_t bloco[4] = {0, 0, 0, 0};

        // Carregar o bloco da memória
        for (int i = 0; i < 16; i += 4) {
            bloco[i / 4] = (MEM8[endereco - (endereco % 16) + i] << 24) | (MEM8[endereco - (endereco % 16) + i + 1] << 16) |
                           (MEM8[endereco - (endereco % 16) + i + 2] << 8) | MEM8[endereco - (endereco % 16) + i + 3];
        }

        // Caso ambas as vias sejam inválidas, usar a primeira
        if (!cache->valid[indice][0]) {
            viaLRU = 0;
        } else if (!cache->valid[indice][1]) {
            viaLRU = 1;
        }

        // Imprimir o log de miss
        sprintf(instrucao, "D_read_miss [%d]", indice);
        fprintf(output, "0x%08X:\t%-25s\t[0]{VAL=%d,AGE=%d,ID=0x%06X},[1]{VAL=%d,AGE=%d,ID=0x%06X}\n", 
                endereco, instrucao, 
                cache->valid[indice][0], cache->lru[indice][0], cache->tag[indice][0],
                cache->valid[indice][1], cache->lru[indice][1], cache->tag[indice][1]);

        // Atualizar cache com o novo bloco
        cache->tag[indice][viaLRU] = tag;
        cache->valid[indice][viaLRU] = 1;
        cache->lru[indice][viaLRU] = 0;
        for (int i = 0; i < 4; i++) {
            cache->data[indice][viaLRU][i] = bloco[i];
        }

        // Retornar o dado apropriado
        if (cacheD.typeMEML == 8) {
            return (bloco[offset] >> (8 * (3 - byteOffset))) & 0xFF;  // Retorna 8 bits
        } else {
            return bloco[offset];  // Retorna 32 bits
        }
    }
}
 
uint32_t leDaCacheI(uint8_t* MEM8, Cache *cache, uint32_t endereco, FILE* output) {
    cacheI.cache_accesses++;
    uint32_t tag = (endereco >> 7);
    uint32_t indice = (endereco >> 4) & 0x07;
    uint32_t offset = (endereco & 0x0F) / 4;
    char instrucao[50] = { 0 };

    int viaLRU = 0, hitVia = -1;

    // Incrementa o LRU de todas as vias válidas da cache
    for (int ind = 0; ind < 8; ind++) {
        for (int via = 0; via < 2; via++) {
            if (cache->valid[ind][via]) {
                if (cache->lru[ind][via] < 255) {
                    cache->lru[ind][via]++;
                }
            }
        }
    }

    // Verificar se há um hit na cache
    for (int i = 0; i < 2; i++) {
        if (cache->valid[indice][i] && cache->tag[indice][i] == tag) {
            hitVia = i;
            break;
        }
        // Guardar a via LRU
        if (cache->lru[indice][i] > cache->lru[indice][viaLRU]) {
            viaLRU = i;
        }
    }

    if (hitVia != -1) {  // Hit na cache
        cacheI.cache_hits++;
        // Atualiza o LRU somente para a via acessada
        cache->lru[indice][hitVia] = 0;

        // Log e retorno dos dados
        sprintf(instrucao, "I_read_hit [%d]->[%d]", indice, hitVia);
        fprintf(output, "0x%08X:\t%-25s\tID=0x%06X,DATA={0x%08X,0x%08X,0x%08X,0x%08X}\n",
                endereco, instrucao, cache->tag[indice][hitVia], cache->data[indice][hitVia][0], cache->data[indice][hitVia][1],
                cache->data[indice][hitVia][2], cache->data[indice][hitVia][3]);

        return cache->data[indice][hitVia][offset];
    } else {  // Miss na cache
        uint32_t bloco[4] = {0, 0, 0, 0};

        // Carregar o bloco da memória
        for (int i = 0; i < 16; i += 4) {
            bloco[i / 4] = (MEM8[endereco - (endereco % 16) + i] << 24) | (MEM8[endereco - (endereco % 16) + i + 1] << 16) |
                            (MEM8[endereco - (endereco % 16) + i + 2] << 8) | MEM8[endereco - (endereco % 16) + i + 3];
        }

        // Caso ambas as vias sejam inválidas, usar a primeira
        if (!cache->valid[indice][0]) {
            viaLRU = 0;
        } else if (!cache->valid[indice][1]) {
            viaLRU = 1;
        }

        // Imprimir o log de miss
        sprintf(instrucao, "I_read_miss [%d]", indice);
        fprintf(output, "0x%08X:\t%-25s\t[0]{VAL=%d,AGE=%d,ID=0x%06X},[1]{VAL=%d,AGE=%d,ID=0x%06X}\n", 
                endereco, instrucao, 
                cache->valid[indice][0], cache->lru[indice][0], cache->tag[indice][0],
                cache->valid[indice][1], cache->lru[indice][1], cache->tag[indice][1]);

        // Atualizar cache com o novo bloco
        cache->tag[indice][viaLRU] = tag;
        cache->valid[indice][viaLRU] = 1;
        memcpy(cache->data[indice][viaLRU], bloco, sizeof(bloco));

        // Resetar a idade da via que foi substituída
        cache->lru[indice][viaLRU] = 0;

        return bloco[offset];
    }
}

// Função para lidar com interrupções
void handleInterrupt(uint8_t* MEM8, uint32_t* R, FILE* output) { // ISR

    escreveNaCacheD(MEM8, &cacheD, R[SP], (R[PC] + 4), output);
    
    R[SP] -= 4; 
    escreveNaCacheD(MEM8, &cacheD, R[SP], R[CR], output);

    R[SP] -= 4; 
    escreveNaCacheD(MEM8, &cacheD, R[SP], R[IPC], output);
    
    R[SP] -= 4;

}

// Função para lidar com uma instrução
void handleInstruction(uint8_t* MEM8, uint32_t* R, FILE* output, uint8_t* executa) {
    char instrucao[50] = { 0 };
    int divii = 0;
    uint8_t z = 0, x = 0, y = 0, v = 0, w = 0;
    int32_t i = 0, offset;
    uint32_t xyl = 0, oldPC, endereco;
    cacheD.typeMEML = 0;
    cacheD.typeMEMS = 0;
    //R[IR] = ((MEM8[R[PC] + 0] << 24) | (MEM8[R[PC] + 1] << 16) | (MEM8[R[PC] + 2] << 8) | (MEM8[R[PC] + 3] << 0));
    R[IR] = leDaCacheI(MEM8, &cacheI, R[PC], output);
    uint8_t opcode = (R[IR] & (0b111111 << 26)) >> 26;

    //Inicializa as funções de string
    char *auxZ = NULL;
    char *auxX = NULL;
    char *auxY = NULL;
    char *auxV = NULL;
    char *auxW = NULL;
    char *auxZUpper = NULL;
    char *auxXUpper = NULL;
    char *auxYUpper = NULL;
    char *auxVUpper = NULL;
    char *auxWUpper = NULL;
    
    switch (opcode) {

        /* INSTRUÇÕES DO TIPO U */
        
        case 0b000000: // mov
            z = (R[IR] & (0b11111 << 21)) >> 21;
            xyl = R[IR] & 0x1FFFFF; // Valor imediato de 21 bits

            // Atualiza o registrador com o valor imediato
            R[z] = xyl;

            // Formata a instrução e imprime o resultado

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);
           
            sprintf(instrucao, "mov %s,%d", auxZ, xyl);
            fprintf(output,"0x%08X:\t%-25s\t%s=0x%08X\n", R[PC], instrucao, auxZUpper, R[z]);
        break;

        case 0b000001: // movs
            z = (R[IR] & (0b11111 << 21)) >> 21;
            xyl = R[IR] & 0x1FFFFF;
        
            // Verifica o bit de sinal (bit mais significativo de 21 bits) e estende o sinal
            if (xyl & (1 << 20)) {
                xyl |= 0xFFE00000; // Extensão de sinal para 32 bits
            }
        
            R[z] = xyl;
        
            // Corrigir a formatação para imprimir o número do registrador corretamente
            sprintf(instrucao, "movs r%u,%d", z, (int32_t)xyl); // Corrigido para mostrar número de registrador e valor como inteiro com sinal
            fprintf(output,"0x%08X:\t%-25s\tR%u=0x%08X\n", R[PC], instrucao, z, R[z]);
        break;

        case 0b000010: // add
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a soma
            int32_t result_add = R[x] + R[y];
            if (z != 0) {
                R[z] = result_add;
            }

            R[SR] &= ~(1 << ZN); 
            R[SR] &= ~(1 << SN);
            R[SR] &= ~(1 << OV);
            R[SR] &= ~(1 << CY);

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << 6);
            }

            // SN Flag
            if (R[z] & (1 << 31)) {
                R[SR] |= (1 << 4);
            }

            if (((R[x] & (1 << 31)) == (R[y] & (1 << 31))) && ((R[z] & (1 << 31)) != (R[x] & (1 << 31)))) {
                R[SR] |= (1 << 3);
            }

            // CY Flag
            // Carry ocorre se o resultado exceder o valor que pode ser representado em 32 bits
            if ((result_add < R[x]) || (result_add < R[y])) {
                R[SR] |= (1 << 0);
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            sprintf(instrucao, "add %s,%s,%s", auxZ, auxX, auxY);
            fprintf(output,"0x%08X:\t%-25s\t%s=%s+%s=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, auxYUpper, R[z], R[SR]);
        break;

        case 0b000011: // sub
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a subtração
            int32_t result_sub = R[x] - R[y];
            R[z] = result_sub;

            R[SR] &= ~(1 << ZN); 
            R[SR] &= ~(1 << SN);
            R[SR] &= ~(1 << OV);
            R[SR] &= ~(1 << CY);

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << ZN);
            }

            // SN Flag
            if ((R[z] >> 31) == 1) {
                R[SR] |= (1 << SN); 
            }

            if (((R[x] & (1 << SR)) != (R[y] & (1 << SR))) && ((R[z] & (1 << SR)) != (R[x] & (1 << SR)))) {
                R[SR] |= (1 << OV);
            }

            // CY Flag
            // Carry ocorre se o resultado é menor que o operando original
            if (R[z] > R[x]) {
                R[SR] |= (1 << CY);
            }

            sprintf(instrucao, "sub r%u,r%u,r%u", z, x, y);
            fprintf(output,"0x%08X:\t%-25s\tR%u=R%u-R%u=0x%08X,SR=0x%08X\n", R[PC], instrucao, z, x, y, R[z], R[SR]);
        break;

        case 0b000101: // cmp
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a subtração
            int32_t result_cmp = R[x] - R[y];

            R[SR] &= ~(1 << ZN); 
            R[SR] &= ~(1 << SN);
            R[SR] &= ~(1 << OV);
            R[SR] &= ~(1 << CY);

            // ZN Flag
            if (result_cmp == 0) {
                R[SR] |= (1 << ZN); 
            }

            // SN Flag
            if (result_cmp & (1 << 31)) {
                R[SR] |= (1 << SN); 
            }

            if (((R[x] & (1 << 31)) != (R[y] & (1 << 31))) && ((result_cmp & (1 << 31)) != (R[x] & (1 << 31)))) {
                R[SR] |= (1 << OV); // OV flag set
            }

            if (result_cmp > R[x]) {
                R[SR] |= (1 << CY); // CY flag set
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            // Formata o registrador x
            sprintf(instrucao, "cmp %s,%s", auxX, auxY);
            fprintf(output,"0x%08X:\t%-25s\tSR=0x%08X\n", R[PC], instrucao, R[SR]);
        break;

        case 0b000110: // and
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a operação AND bit a bit
            if (z != 0) {
                R[z] = R[x] & R[y];
            }

            // Ajusta o SR com base nos campos afetados
            R[SR] &= ~(1 << ZN); // ZN flag off
            R[SR] &= ~(1 << SN); // SN flag off

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << ZN); 
            }

            // SN Flag
            if (R[z] & (1 << 31)) {
                R[SR] |= (1 << SN); 
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            sprintf(instrucao, "and %s,%s,%s", auxZ, auxX, auxY);
            fprintf(output,"0x%08X:\t%-25s\t%s=%s&%s=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, auxYUpper, R[z], R[SR]);
        break;

        case 0b000111: // or
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a operação OR bit a bit
            if (z != 0) {
                R[z] = R[x] | R[y];
            }

            // Ajusta o SR com base nos campos afetados
            R[SR] &= ~(1 << ZN); // ZN flag off
            R[SR] &= ~(1 << SN);

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << ZN); 
            }

            // SN Flag
            if (R[z] & (1 << 31)) {
                R[SR] |= (1 << SN); 
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            sprintf(instrucao, "or %s,%s,%s", auxZ, auxX, auxY);
            fprintf(output,"0x%08X:\t%-25s\t%s=%s|%s=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, auxYUpper, R[z], R[SR]);
        break;

        case 0b001000: // not
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;

            // Executa a operação NOT bit a bit
            if (z != 0) {
                R[z] = ~R[x];
            }

            // Ajusta o SR com base nos campos afetados
             R[SR] &= ~(1 << ZN); // ZN flag off
             R[SR] &= ~(1 << SN); // SN flag off

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << ZN); 
            }

            // SN Flag
            if (R[z] & (1 << 31)) {
                R[SR] |= (1 << SN); 
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            sprintf(instrucao, "not %s,%s", auxZ, auxX);
            fprintf(output,"0x%08X:\t%-25s\t%s=~%s=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, R[z], R[SR]);
        break;

        case 0b001001: // xor
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;

            // Executa a operação XOR bit a bit
            if(z != 0) {
                R[z] = R[x] ^ R[y];
            }

            R[SR] &= ~(1 << ZN); // ZN flag off
            R[SR] &= ~(1 << SN); // SN flag off

            // ZN Flag
            if (R[z] == 0) {
                R[SR] |= (1 << ZN); 
            }

            // SN Flag
            if (R[z] & (1 << 31)) {
                R[SR] |= (1 << SN); 
            }

            auxZ = registerNameLower(z);
            auxX = registerNameLower(x);
            auxY = registerNameLower(y);
            auxZUpper = registerNameUpper(z);
            auxXUpper = registerNameUpper(x);
            auxYUpper = registerNameUpper(y);

            sprintf(instrucao, "xor %s,%s,%s", auxZ, auxX, auxY);
            fprintf(output,"0x%08X:\t%-25s\t%s=%s^%s=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, auxYUpper, R[z], R[SR]);
        break;

                // PADRÃO DOS OPERADORES
        case 0b000100: // Código de operação comum para os 8 operadores
            z = (R[IR] & (0b11111 << 21)) >> 21;
            x = (R[IR] & (0b11111 << 16)) >> 16;
            y = (R[IR] & (0b11111 << 11)) >> 11;
            int32_t l = R[IR] & 0x7FF;
            uint8_t top3Bits = (l >> 8) & 0x07;
            uint8_t L_restante = l & 0x1F;
            uint64_t result = 0;
            uint64_t resultAux = 0;

            int64_t result_int = 0;
            int64_t resultAux_int = 0;
            
           // Identificação do operador específico com base nos últimos 11 bits
            switch (top3Bits) {
                case 0b011: // SLA
                    resultAux_int = ((int64_t)(int32_t)R[z] << 32) | (int64_t)(int32_t)R[y];
                    result_int = resultAux_int << (L_restante + 1);
                    // Armazena os primeiros 32 bits de 'result' no R[z]
                    if (z != 0) {
                        R[z] = (int32_t)(result_int >> 32);
                    }
                    R[x] = (int32_t)(result_int);
                    R[SR] &= ~(1 << ZN); 
                    R[SR] &= ~(1 << OV);

                    if (result_int == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    }
                    if (R[z] != 0) {
                        R[SR] |= (1 << OV);
                    }
                    // Impressão dos resultados para o 'sla'
                    
                    sprintf(instrucao, "sla r%u,r%u,r%u,%u", z, x, y, L_restante);
                    fprintf(output,"0x%08X:\t%-25s\tR%u:R%u=R%u:R%u<<%d=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           z,          // Registrador z
                           x,          // Registrador x
                           z,          // Registrador z (para R[z] no resultado)
                           x,          // Registrador x (para R[x] no resultado)
                           L_restante + 1, // Valor de l
                           ((int64_t)R[z] << 32) | R[x], // Resultado da operação
                           R[SR]       // SR
                    );
                break;
                    
                case 0b000: // MUL
                    result = (uint64_t)R[x] * (uint64_t)R[y];
                    
                    if (L_restante != 0) {
                        R[L_restante] = (uint32_t)(result >> 32);
                    }
                    // Armazena o resultado no registrador z
                    if (z != 0) {
                        R[z] = (uint32_t)result;
                    }
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << CY); // CY flag off
                    // Atualiza os flags no registrador de status
                    if (result == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    }
        
                    // Define a flag CY se o resultado exceder 32 bits
                    if (R[L_restante] != 0) {
                        R[SR] |= (1 << CY); // Define a flag CY se houver carry
                    }
                    // Impressão dos resultados para o 'mul'
                    sprintf(instrucao, "mul r%u,r%u,r%u,r%u",L_restante, z, x, y);
                    fprintf(output,"0x%08X:\t%-25s\tR%u:R%u=R%u*R%u=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           L_restante,          // Registrador z
                           z,          // Registrador x
                           x,          // Registrador z (para R[z] no resultado)
                           y,          // Registrador x (para R[x] no resultado)
                           ((uint64_t)R[L_restante] << 32) | R[z], // Resultado da operação
                           R[SR]       // SR
                    );
                    break;
                case 0b001: // SLL
                    
                    result = ((uint64_t)R[z] << 32) | (uint64_t)R[y];
                    result = result << (L_restante + 1);
                    // Armazena os primeiros 32 bits de 'result' no R[z]
                    if (z != 0) {
                        R[z] = (uint32_t)(result >> 32);
                    }
                    // Armazena os últimos 32 bits de 'result' no R[x]
                    R[x] = (uint32_t)(result);
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << CY); // CY flag off
                    // Atualiza os flags no registrador de status
                    if (result == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    } 
                    // Define a flag CY com base no valor de R[z]
                    if (R[z] != 0) {
                        R[SR] |= (1 << CY); // Define a flag CY se R[z] for diferente de zero
                    } 
                    // Impressão dos resultados para o 'sll'
                    sprintf(instrucao, "sll r%u,r%u,r%u,%u", z, x, y, L_restante);
                    fprintf(output,"0x%08X:\t%-25s\tR%u:R%u=R%u:R%u<<%d=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           z,          // Registrador z
                           x,          // Registrador x
                           z,          // Registrador z (para R[z] no resultado)
                           x,          // Registrador x (para R[x] no resultado)
                           L_restante + 1, // Valor de l
                           ((uint64_t)R[z] << 32) | R[x], // Resultado da operação
                           R[SR]       // SR
                    );
                    break;
                case 0b010: // MULS
                    // Multiplicação com sinal
                    result_int = (int64_t)(int32_t)R[x] * (int64_t)(int32_t)R[y];
                
                    if (L_restante != 0) {
                        R[L_restante] = (int32_t)(result_int >> 32); // Parte alta
                    }
                    // Armazena o resultado no registrador z
                    if (z != 0) {
                        R[z] = (int32_t)result_int; // Parte baixa
                    }
                
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << OV); // OV flag off
                
                    // Atualiza os flags no registrador de status
                    if (R[z] == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    }
                
                    // Overflow ocorre se a parte alta não é zero (resulta em um número fora dos limites de 32 bits)
                    if (R[L_restante] != 0) {
                        R[SR] |= (1 << OV); // Define a flag OV se houver overflow
                    }
                
                    // Impressão dos resultados para o 'muls'
                    sprintf(instrucao, "muls r%u,r%u,r%u,r%u", L_restante, z, x, y);
                    fprintf(output, "0x%08X:\t%-25s\tR%u:R%u=R%u*R%u=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           L_restante, // Registrador l4−0
                           z,          // Registrador z
                           x,          // Registrador x
                           y,          // Registrador y
                           ((int64_t)R[L_restante] << 32) | R[z],     // Resultado da operação
                           R[SR]       // SR
                    );
                    break;
                case 0b100: // DIV
                    R[SR] &= ~(1 << ZD); // ZD flag off
                    R[SR] &= ~(1 << ZN); // ZN flag off

                    // Calcula a divisão e o módulo
                    result = (uint64_t)R[x] / (uint64_t)R[y];
                    resultAux = (uint64_t)R[x] % (uint64_t)R[y];
                
                    oldPC = R[PC];
                    divii = 0;
                    // Divisão e módulo
                    if (R[y] == 0) {
                        R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
                        divii = 1;
                    } else {
                        R[SR] &= ~(1 << CY);
                        // Calcula a divisão e o módulo
                
                        if (z != 0) {
                            R[z] = (uint32_t)result; // Armazena o quociente
                        }
                        if (L_restante != 0) {
                            R[L_restante] = (uint32_t)resultAux;
                        }
                        if (R[z] == 0) {
                            R[SR] |= (1 << ZN); // Define a flag ZN se o quociente for zero
                        }
                
                        if (R[L_restante] != 0) {
                            R[SR] |= (1 << CY); // Define a flag CY se o módulo for zero
                        }
                    }
                            // Impressão dos resultados para o 'div'
                        sprintf(instrucao, "div r%u,r%u,r%u,r%u", L_restante, z, x, y);
                        fprintf(output,"0x%08X:\t%-25s\tR%u=R%u%%R%u=0x%08X,R%u=R%u/R%u=0x%08X,SR=0x%08X\n",
                               R[PC],      
                               instrucao,  
                               L_restante,          // Registrador z (quociente)
                               x,          // Registrador x (dividendo)
                               y,          // Registrador y (divisor)
                               R[L_restante],       // Quociente
                               z, // Registrador l (módulo)
                               x,          // Registrador x (dividendo)
                               y,          // Registrador y (divisor)
                               R[z], // Módulo
                               R[SR]       // SR
                        );

                        if (divii == 1) { // Verifica o bit IE em SR
                            // IE = 1
                            R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
                            fprintf(output,"[SOFTWARE INTERRUPTION]\n");
                            handleInterrupt(MEM8, R, output);
                            R[CR] = 0;
                            R[IPC] = R[PC];
                            R[PC] = 0x00000004;
                        }
                break;

                case 0b101: // SRL
                    result = ((uint64_t)R[z] >> 32) | (uint64_t)R[y];
                    result = result >> (L_restante + 1);
                    // Armazena os primeiros 32 bits de 'result' no R[z]
                    if (z != 0) {
                        R[z] = (uint32_t)(result >> 32);
                    }
                    // Armazena os últimos 32 bits de 'result' no R[x]
                    R[x] = (uint32_t)(result);
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << CY); // CY flag off
                    // Atualiza os flags no registrador de status
                    if (result == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    } 
                    // Define a flag CY com base no valor de R[z]
                    if (R[z] != 0) {
                        R[SR] |= (1 << CY); // Define a flag CY se R[z] for diferente de zero
                    } 
                    // Impressão dos resultados para o 'sll'
                    sprintf(instrucao, "srl r%u,r%u,r%u,%u", z, x, y, L_restante);
                    fprintf(output,"0x%08X:\t%-25s\tR%u:R%u=R%u:R%u>>%d=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           z,          // Registrador z
                           x,          // Registrador x
                           z,          // Registrador z (para R[z] no resultado)
                           x,          // Registrador x (para R[x] no resultado)
                           L_restante + 1, // Valor de l
                           result, // Resultado da operação
                           R[SR]       // SR
                    );
                    break;
                case 0b110: // DIVS
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << OV); // CY flag off
                    R[SR] &= ~(1 << ZD);
                    divii = 0;
                    // Divisão e módulo
                    if (R[z] == 0) {                                                
                        R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
        
                    } else {
                         // Calcula a divisão e o módulo
                        result_int = (int64_t)(int32_t)R[x] / (int64_t)(int32_t)R[y];
                        resultAux_int = (int64_t)(int32_t)R[x] % (int64_t)(int32_t)R[y];
                
                        // Armazena os resultados
                        if (z != 0) {
                            R[z] = (int32_t)result_int; // Armazena o quociente
                        }
                        R[L_restante] = (int32_t)resultAux_int; // Armazena o módulo
                    
                        // Atualiza os flags no registrador de status
                        if (R[z] == 0) {
                            R[SR] |= (1 << ZN); // Define a flag ZN se o quociente for zero
                        }
                
                        if (resultAux_int != 0) {
                            R[SR] |= (1 << OV); // Define a flag CY se o módulo for zero
                        }
                            // Impressão dos resultados para o 'divs'
                        sprintf(instrucao, "divs r%u,r%u,r%u,r%u", L_restante, z, x, y);
                        fprintf(output,"0x%08X:\t%-25s\tR%u=R%u%%R%u=0x%08X,R%u=R%u/R%u=0x%08X,SR=0x%08X\n",
                               R[PC],      
                               instrucao,  
                               L_restante,   // Registrador z (quociente)
                               x,          // Registrador x (dividendo)
                               y,          // Registrador y (divisor)
                               R[L_restante],       // Quociente
                               z, // Registrador l (módulo)
                               x,          // Registrador x (dividendo)
                               y,          // Registrador y (divisor)
                               R[z], // Módulo
                               R[SR]       // SR
                        );
                        if ((R[SR] & (IE << 1)) != 0) { // Verifica o bit IE em SR
                            // IE = 1
                            handleInterrupt(MEM8, R, output);
                            R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
                            R[CR] = 0;
                            R[IPC] = R[PC];
                            R[PC] = 0x00000004;
                        }
                    }
                break;

                case 0b111: // SRA
                    resultAux_int = ((uint64_t)(uint32_t)R[z] << 32) | (uint64_t)(uint32_t)R[y];
                    result_int = (int64_t)resultAux_int >> (L_restante + 1);

                    // Armazena os primeiros 32 bits de 'result_int' no R[z]
                    if (z != 0) {
                        R[z] = (int32_t)(result_int >> 32);
                    }
                    // Armazena os últimos 32 bits de 'result_int' no R[x]'
                    R[x] = (int32_t)result_int;
                    // Atualiza os flags no registrador de status
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[SR] &= ~(1 << OV); // OV flag off
                    
                    if ((((uint64_t)R[z] << 32) | R[x]) == 0) {
                        R[SR] |= (1 << ZN); // Define a flag ZN se o resultado for zero
                    }
                    if(R[z] != 0) {
                        R[SR] |= (1 << OV);
                    }
                    // Impressão dos resultados para o 'sra'
                    sprintf(instrucao, "sra r%u,r%u,r%u,%u", z, x, y, L_restante);
                    fprintf(output, "0x%08X:\t%-25s\tR%u:R%u=R%u:R%u>>%d=0x%016" PRIX64 ",SR=0x%08X\n",
                           R[PC],      
                           instrucao,  
                           z,          // Registrador z
                           x,          // Registrador x
                           z,          // Registrador z (para R[z] no resultado)
                           x,          // Registrador x (para R[x] no resultado)
                           L_restante + 1, // Valor de l
                           ((uint64_t)R[z] << 32) | R[x],     // Resultado da operação
                           R[SR]       // SR
                    );
                    break;
            }
            break;

            case 0b001010: // push
                z = (R[IR] & (0b11111 << 21)) >> 21;

                x = (R[IR] & (0b11111 << 16)) >> 16;
                y = (R[IR] & (0b11111 << 11)) >> 11;
                v = (R[IR] & (0b11111 << 6)) >> 6;
                w = R[IR] & 0b11111;
                uint32_t ipush[] = {v, w, x, y, z};
                int posicao = 0;
                

                for (posicao = 0; posicao < 5; posicao++) {
                    if (ipush[posicao] != 0) {
                        
                        escreveNaCacheD(MEM8, &cacheD, R[SP], R[ipush[posicao]], output);
                        
                        R[SP] -= 4;
                    }
                    else {
                        break;
                    }
                }

                auxZ = registerNameLower(z);
                auxX = registerNameLower(x);
                auxY = registerNameLower(y);
                auxV = registerNameLower(v);
                auxW = registerNameLower(w);
                auxZUpper = registerNameUpper(z);
                auxXUpper = registerNameUpper(x);
                auxYUpper = registerNameUpper(y);
                auxVUpper = registerNameUpper(v);
                auxWUpper = registerNameUpper(w);

                if (posicao == 0) {
                    

                    sprintf(instrucao, "push -"); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{}={}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP]);                  
                                                                              
                } else if (posicao == 1) {
                    
                    sprintf(instrucao, "push %s", auxV); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{0x%08X}={%s}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP] + 4,                                           
                            R[ipush[0]],
                            auxVUpper);
                            
                } else if (posicao == 2) {
                    
                    sprintf(instrucao, "push %s,%s", auxV, auxW); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{0x%08X,0x%08X}={%s,%s}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP] + 8,                                           
                            R[ipush[0]],R[ipush[1]],
                            auxVUpper, auxWUpper);
                } else if (posicao == 3) {
                    

                    sprintf(instrucao, "push %s,%s,%s", auxV, auxW, auxX); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{0x%08X,0x%08X,0x%08X}={%s,%s,%s}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP] + 12,                                           
                            R[ipush[0]],R[ipush[1]],R[ipush[2]],
                            auxVUpper, auxWUpper, auxXUpper);
                } else if (posicao == 4) {
                    

                    sprintf(instrucao, "push %s,%s,%s,%s", auxV, auxW, auxX, auxY); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{0x%08X,0x%08X,0x%08X,0x%08X}={%s,%s,%s,%s}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP] + 16,                                           
                            R[ipush[0]],R[ipush[1]],R[ipush[2]],R[ipush[3]],
                            auxVUpper, auxWUpper, auxXUpper, auxYUpper);
                } else if (posicao == 5) {
                    

                    sprintf(instrucao, "push %s,%s,%s,%s,%s", auxV, auxW, auxX, auxY, auxZ); 
                    
                    fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]{0x%08X,0x%08X,0x%08X,0x%08X,0x%08X}={%s,%s,%s,%s,%s}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP] + 20,                                           
                            R[ipush[0]],R[ipush[1]],R[ipush[2]],R[ipush[3]],R[ipush[4]],
                            auxVUpper, auxWUpper, auxXUpper, auxYUpper, auxZUpper);
                }
                break;
            case 0b001011: // pop
                z = (R[IR] & (0b11111 << 21)) >> 21;

                x = (R[IR] & (0b11111 << 16)) >> 16;
                y = (R[IR] & (0b11111 << 11)) >> 11;
                v = (R[IR] & (0b11111 << 6)) >> 6;
                w = R[IR] & 0b11111;
                uint32_t ipop[] = {v, w, x, y, z};
                int posicaoPop = 0;

                for (posicaoPop = 0; posicaoPop < 5; posicaoPop++) {
                    if (ipop[posicaoPop] != 0) {

                        R[SP] += 4;
                        R[ipop[posicaoPop]] = leDaCacheD(MEM8, &cacheD, R[SP], output);
                    }
                    else {
                        break;
                    }
                }

                auxZ = registerNameLower(z);
                auxX = registerNameLower(x);
                auxY = registerNameLower(y);
                auxV = registerNameLower(v);
                auxW = registerNameLower(w);
                auxZUpper = registerNameUpper(z);
                auxXUpper = registerNameUpper(x);
                auxYUpper = registerNameUpper(y);
                auxVUpper = registerNameUpper(v);
                auxWUpper = registerNameUpper(w);

                if (posicaoPop == 0) {
                    

                    sprintf(instrucao, "pop -"); // Formata o registrador como "r
                    
                    fprintf(output, "0x%08X:\t%-25s\t{}=MEM[0x%08X]{}\n", 
                            R[PC],                      
                            instrucao,                  
                            R[SP]);                  
                                                                              
                } else if (posicaoPop == 1) {
                    
                    sprintf(instrucao, "pop %s", auxV); 
                    
                    fprintf(output, "0x%08X:\t%-25s\t{%s}=MEM[0x%08X]{0x%08X}\n", 
                            R[PC],                      
                            instrucao,
                            auxVUpper,                 
                            R[SP] - 4,                                          
                            R[ipop[0]]);
                            
                } else if (posicaoPop == 2) {
                    
                    sprintf(instrucao, "pop %s,%s", auxV, auxW); 
                    
                    fprintf(output, "0x%08X:\t%-25s\t{%s,%s}=MEM[0x%08X]{0x%08X,0x%08X}\n", 
                            R[PC],                      
                            instrucao,
                            auxVUpper, auxWUpper,             
                            R[SP] - 8,                                           
                            R[ipop[0]],R[ipop[1]]);

                } else if (posicaoPop == 3) {
                    
                    sprintf(instrucao, "pop %s,%s,%s", auxV, auxW, auxX); 
                    
                    fprintf(output, "0x%08X:\t%-25s\t{%s,%s,%s}=MEM[0x%08X]{0x%08X,0x%08X,0x%08X}\n", 
                            R[PC],                      
                            instrucao,
                            auxVUpper, auxWUpper, auxXUpper,               
                            R[SP] - 12,                                           
                            R[ipop[0]],R[ipop[1]],R[ipop[2]]);

                } else if (posicaoPop == 4) {
                    
                    sprintf(instrucao, "pop %s,%s,%s,%s", auxV, auxW, auxX, auxY); 
                    
                    fprintf(output, "0x%08X:\t%-25s\t{%s,%s,%s,%s}=MEM[0x%08X]{0x%08X,0x%08X,0x%08X,0x%08X}\n", 
                            R[PC],                      
                            instrucao,
                            auxVUpper, auxWUpper, auxXUpper, auxYUpper,               
                            R[SP] - 16,                                           
                            R[ipop[0]],R[ipop[1]],R[ipop[2]],R[ipop[3]]);

                } else if (posicaoPop == 5) {
                    
                    sprintf(instrucao, "pop %s,%s,%s,%s,%s", auxV, auxW, auxX, auxY, auxZ); 
                    
                    fprintf(output, "0x%08X:\t%-25s\t{%s,%s,%s,%s,%s}=MEM[0x%08X]{0x%08X,0x%08X,0x%08X,0x%08X,0x%08X}\n", 
                            R[PC],                      
                            instrucao,
                            auxVUpper, auxWUpper, auxXUpper, auxYUpper, auxZUpper,           
                            R[SP] - 20,                                           
                            R[ipop[0]],R[ipop[1]],R[ipop[2]],R[ipop[3]],R[ipop[4]]);
                }

                break;

                /* INSTRUÇÕES DO TIPO S */

            case 0b101010: // bae
                // Extração do campo imediato de 26 bits da instrução
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (AE ← CY = 0)
                if ((R[SR] & (1 << CY)) == 0) { // Verifica se o Carry Flag (CY) é 0
                    // Calcular o novo endereço do PC
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                
                sprintf(instrucao, "bae %d", i);
                
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b101011: // bat
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                if (((R[SR] & (1 << ZN)) == 0) && ((R[SR] & (1 << CY)) == 0)) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 
                sprintf(instrucao, "bat %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;
        
            case 0b101100: // bbe
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (BE ← (ZN = 1 ∨ CY = 1))
                if (((R[SR] & (1 << ZN)) != 0) || ((R[SR] & (1 << CY)) != 0)) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "bbe %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b101101: // bbt
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (BT ← CY = 1)
                if ((R[SR] & (1 << CY)) != 0) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "bbt %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b101110: // beq
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (EQ ← ZN = 1)
                if ((R[SR] & (1 << ZN)) != 0) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "beq %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b101111: // bge
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (GE ← SN = OV)
                if (((R[SR] & (1 << SN)) >> SN) == ((R[SR] & (1 << OV)) >> OV)) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "bge %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110000: // bgt
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (GT ← (ZN = 0 ∧ SN = OV))
                if (((R[SR] & (1 << ZN)) == 0) && (((R[SR] & (1 << SN)) >> SN) == ((R[SR] & (1 << OV)) >> OV))) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "bgt %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110001: // biv
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (IV ← PC=PC+4+6 i25:i<<2)
                if (((R[SR] & (1 << 2)) >> 2) != 0) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                sprintf(instrucao, "biv %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110010: // ble
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (LE ← (ZN = 1 ∨ SN = OV))
                if (((R[SR] & (1 << ZN)) != 0) || (((R[SR] & (1 << SN)) >> SN) != ((R[SR] & (1 << OV)) >> OV))) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "ble %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110011: // blt
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (LT ← SN = OV)
                if (((R[SR] & (1 << SN)) >> SN) != ((R[SR] & (1 << OV)) >> OV)) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                sprintf(instrucao, "blt %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110100: // bne
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (NE ← ZN = 0)
                if ((R[SR] & (1 << ZN)) == 0) {
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                } 

                sprintf(instrucao, "bne %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110101: // bni
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (NI ← IV = 0)
                if ((R[SR] & (1 << IV)) == 0) { // Verifica se o IV é 0
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                sprintf(instrucao, "bni %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110110: // bnz
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                // Verificar a condição de desvio (NZ ← ZD = 0)
                if ((R[SR] & (1 << ZD)) == 0) { // Verifica se o ZD é 0
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                sprintf(instrucao, "bnz %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b110111: //bun
                i = R[IR] & 0x03FFFFFF;

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                oldPC = R[PC];

                offset = i << 2;
                R[PC] = oldPC + offset;

                sprintf(instrucao, "bun %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b111000: // bzd
                i = R[IR] & 0x03FFFFFF; 
                oldPC = R[PC];

                if (i & (1 << 25)) { 
                    i |= 0xFC000000; 
                }

                if ((R[SR] & (1 << ZD)) != 0) { 
                    offset = i << 2; 
                    R[PC] = oldPC + offset;
                }

                sprintf(instrucao, "bzd %d", i);
                fprintf(output,"0x%08X:\t%-25s\tPC=0x%08X\n", oldPC, instrucao, R[PC] + 4);
            break;

            case 0b111111: //int 0
                i = R[IR] & 0x03FFFFFF;
                oldPC = R[PC];

                if (i == 0) {
                    *executa = 0;
                    sprintf(instrucao, "int %u", i);
                    fprintf(output,"0x%08X:\t%-25s\tCR=0x00000000,PC=0x00000000\n", oldPC, instrucao);
                    cacheD.D_hit_rate = (cacheD.cache_accesses > 0) ? ((float)cacheD.cache_hits / cacheD.cache_accesses) * 100 : 0.0;
                    cacheI.I_hit_rate = (cacheI.cache_accesses > 0) ? ((float)cacheI.cache_hits / cacheI.cache_accesses) * 100 : 0.0;
                    fprintf(output, "[CACHE]\n");
                    fprintf(output, "D_hit_rate: %.2f%%\n", cacheD.D_hit_rate);
                    fprintf(output, "I_hit_rate: %.2f%%\n", cacheI.I_hit_rate);
                    if (terminal.bufferIndex > 0) {
                        fprintf(output,"[TERMINAL] \n%s\n", terminal.buffer);
                    }
                } else {
                    sprintf(instrucao, "int %u", i);
                    fprintf(output,"0x%08X:\t%-25s\tCR=0x%08X,PC=0x%08X\n", oldPC, instrucao, i, 0x0000000C);
                    fprintf(output,"[SOFTWARE INTERRUPTION]\n");
                    handleInterrupt(MEM8, R, output);
                    R[CR] = i;
                    R[IPC] = R[PC];
                    R[PC] = 0x00000008;
                    
                }

            break;

            case 0b111001: // CALL S
                i = R[IR] & 0x03FFFFFF; // Campo imediato i (26 bits)

                int32_t i_calls = (int32_t)(int16_t)i;

                oldPC = R[PC];

                escreveNaCacheD(MEM8, &cacheD, R[SP], (R[PC] + 4), output);
                
                R[SP] -= 4;

                // Calcular o novo valor do PC
                offset = i_calls << 2; // Desloca o campo imediato i para obter o offset
                R[PC] = oldPC + offset; // Define o novo valor de PC

                
                sprintf(instrucao, "call %d", i_calls); // Formata o campo imediato como um número decimal

                
                fprintf(output, "0x%08X:\t%-25s\tPC=0x%08X,MEM[0x%08X]=0x%08X\n", 
                        oldPC,                       
                        instrucao,                   
                        R[PC] + 4,                       // Novo valor do PC
                        R[SP] + 4,                   // Endereço onde o valor foi armazenado
                        oldPC + 4);
            break;

                /* INSTRUÇÕES DO TIPO F */

            case 0b010010: // addi
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_addi = R[IR] & 0xFFFF; 
                int32_t i_addi = (int32_t)(int16_t)immediate_addi; 

                int32_t result_addi = R[x] + i_addi;
                R[z] = result_addi;

                R[SR] &= ~(1 << ZN); 
                R[SR] &= ~(1 << SN);
                R[SR] &= ~(1 << OV);
                R[SR] &= ~(1 << CY);

               // ZN Flag
                if (R[z] == 0) {
                    R[SR] |= (1 << ZN); 
                }

                // SN Flag
                if (R[z] & (1 << 31)) {
                    R[SR] |= (1 << SN); 
                }
    
                // OV Flag
                int32_t x_sign_addi = R[x] & (1 << 31); 
                int32_t i_sign_addi = i_addi & (1 << 31); 
                int32_t z_sign_addi = R[z] & (1 << 31); 
                if ((x_sign_addi == i_sign_addi) && (z_sign_addi != x_sign_addi)) {
                    R[SR] |= (1 << OV); 
                }

                // CY Flag
                if (R[z] < R[x]) {
                    R[SR] |= (1 << CY); 
                }

                sprintf(instrucao, "addi r%u,r%u,%i", z, x, i_addi);
                fprintf(output,"0x%08X:\t%-25s\tR%u=R%u+0x%08X=0x%08X,SR=0x%08X\n", R[PC], instrucao, z, x, i_addi, R[z], R[SR]);
            break;

            case 0b010011: // subi
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_subi = R[IR] & 0xFFFF; 
                int32_t i_subi = (int32_t)(int16_t)immediate_subi; 

                int32_t result_subi = R[x] - i_subi;
                R[z] = result_subi;

                R[SR] &= ~(1 << ZN); 
                R[SR] &= ~(1 << SN);
                R[SR] &= ~(1 << OV);
                R[SR] &= ~(1 << CY);

                // ZN Flag
                if (R[z] == 0) {
                    R[SR] |= (1 << ZN); 
                }

                // SN Flag
                if (R[z] & (1 << 31)) {
                    R[SR] |= (1 << SN); 
                }

                // OV Flag
                int32_t x_sign_subi = R[x] & (1 << 31); 
                int32_t i_sign_subi = i_subi & (1 << 31); 
                int32_t z_sign_subi = R[z] & (1 << 31);
                if ((x_sign_subi != i_sign_subi) && (z_sign_subi != x_sign_subi)) {
                    R[SR] |= (1 << OV); 
                }

                // CY Flag
                if (R[z] > R[x]) {
                    R[SR] |= (1 << CY); 
                }

                sprintf(instrucao, "subi r%u,r%u,%i", z, x, i_subi);
                fprintf(output,"0x%08X:\t%-25s\tR%u=R%u-0x%08X=0x%08X,SR=0x%08X\n", R[PC], instrucao, z, x, i_subi, R[z], R[SR]);
            break;

            case 0b010100: //muli
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_muli = R[IR] & 0xFFFF; 
                int32_t i_muli = (int32_t)(int16_t)immediate_muli; 

                int64_t result_muli = (int64_t)(int32_t)R[x] * i_muli; 
                R[z] = (int32_t)result_muli; 

                R[SR] &= ~(1 << 6); // ZN flag off
                R[SR] &= ~(1 << 3); // OV flag off

                // ZN Flag
                if (R[z] == 0) {
                    R[SR] |= (1 << ZN);
                }

                // OV Flag
                // Se a parte alta do resultado for zero, não houve overflow
                if ((R[z] >= 32) != 0) {
                    R[SR] |= (1 << OV);
                }

                sprintf(instrucao, "muli r%u,r%u,%i", z, x, i_muli);
                fprintf(output,"0x%08X:\t%-25s\tR%u=R%u*0x%08X=0x%08X,SR=0x%08X\n", R[PC], instrucao, z, x, i_muli, R[z], R[SR]);
            break;

            case 0b010101: // divi
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_divi = R[IR] & 0xFFFF; 
                int32_t i_divi = (int32_t)(int16_t)immediate_divi; 
                divii = 0;

                int64_t result_divi = (int64_t)(int32_t)R[x] / i_divi;

                // Ajusta o SR com base nos campos afetados
                R[SR] &= ~(1 << ZD); // ZD flag off
                R[SR] &= ~(1 << OV);

                // Verifica se o divisor é zero e atualiza as flags
                if (i_divi == 0) { 
                    R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
                    divii = 1;
                        
                } else {
                    R[SR] &= ~(1 << ZN); // ZN flag off
                    R[z] = (int32_t)result_divi;

                     // ZN Flag
                    if (R[z] == 0) {
                        R[SR] |= (1 << ZN);
                    } 
                }

                auxZ = registerNameLower(z);
                auxX = registerNameLower(x);
                auxY = registerNameLower(y);
                auxZUpper = registerNameUpper(z);
                auxXUpper = registerNameUpper(x);
                auxYUpper = registerNameUpper(y);

                sprintf(instrucao, "divi %s,%s,%i", auxZ, auxX, i_divi);
                fprintf(output,"0x%08X:\t%-25s\t%s=%s/0x%08X=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, i_divi, R[z], R[SR]);

                if (divii == 1) { // Verifica o bit IE em SR
                    // IE = 1
                    fprintf(output,"[SOFTWARE INTERRUPTION]\n");
                    handleInterrupt(MEM8, R, output);
                    R[SR] |= (1 << ZD); // Define ZD (flag de divisão por zero) para 1
                    R[CR] = 0;
                    R[IPC] = R[PC];
                    R[PC] = 0x00000004;
                }
            break;

            case 0b010110: // modi
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_modi = R[IR] & 0xFFFF; 
                int32_t i_modi = (int32_t)(int16_t)immediate_modi; 
                // Ajusta o SR com base nos campos afetados
                R[SR] &= ~(1 << ZN); 
                R[SR] &= ~(1 << ZD); 
                R[SR] &= ~(1 << OV);
                // Verifica se o divisor é zero e atualiza as flags
                if (i_modi == 0) {
                    R[SR] |= (1 << ZD); 
                    // Não faz a operação se o divisor for zero
                } else {
                    // Realiza a operação módulo
                    int64_t result_modi = (int64_t)(int32_t)R[x] % i_modi;
                    R[z] = (int32_t)result_modi;
                }
                // ZN Flag
                    if (R[z] == 0) {
                        R[SR] |= (1 << ZN);
                    }

                auxZ = registerNameLower(z);
                auxX = registerNameLower(x);
                auxY = registerNameLower(y);
                auxZUpper = registerNameUpper(z);
                auxXUpper = registerNameUpper(x);
                auxYUpper = registerNameUpper(y);
                    
                sprintf(instrucao, "modi %s,%s,%i", auxZ, auxX, i_modi);
                fprintf(output,"0x%08X:\t%-25s\t%s=%s%%0x%08X=0x%08X,SR=0x%08X\n", R[PC], instrucao, auxZUpper, auxXUpper, i_modi, R[z], R[SR]);
            break;

            case 0b010111: // cmpi
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                int16_t immediate_cmpi = R[IR] & 0xFFFF; 
                int32_t i_cmpi = (int32_t)(int16_t)immediate_cmpi; 

                int64_t result_cmpi = (int64_t)(int32_t)R[x] - i_cmpi;

                // Ajusta o SR com base nos campos afetados
                R[SR] &= ~(1 << ZN); 
                R[SR] &= ~(1 << SN); 
                R[SR] &= ~(1 << OV); 
                R[SR] &= ~(1 << CY); 

                // ZN Flag
                if (result_cmpi == 0) {
                    R[SR] |= (1 << ZN); 
                }

                // SN Flag
                if ((result_cmpi >> 31) & 1) {
                    R[SR] |= (1 << SN); 
                }

                // OV Flag
                if (((R[x] >> 31) != (i_cmpi >> 15)) && ((result_cmpi >> 31) != (R[x] >> 31))) {
                    R[SR] |= (1 << OV); 
                }

                // CY Flag
                if (result_cmpi < 0) {
                    R[SR] |= (1 << CY); 
                }

                sprintf(instrucao, "cmpi r%u,%i", x, i_cmpi);
                fprintf(output,"0x%08X:\t%-25s\tSR=0x%08X\n", R[PC], instrucao, R[SR]);
            break;

            // l8
            case 0b011000:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = R[x] + i;
                
                if (endereco == 0x80808080) {
                    // Leitura do registrador do watchdog
                    R[z] = ((wd.EN & 0x1) << 31) | (wd.COUNTER & 0x7FFFFFFF);
                } 
                else if (endereco == 0x80808880) {
                    R[z] = fpu.X;
                }
                else if (endereco == 0x80808884) {
                    R[z] = fpu.Y;
                }
                else if (endereco == 0x80808888) {
                    R[z] = fpu.Z;
                }
                else if (endereco == 0x8080888F) {
                    R[z] = ((fpu.ST & 0x1) << 5) | (fpu.OP & 0x1F);
                } else {
                    // Leitura normal da memória

                    cacheD.typeMEML = 8;
                    R[z] = leDaCacheD(MEM8, &cacheD, endereco, output);
                    
                }

                
                sprintf(instrucao, "l8 r%u,[r%u%s%i]", z, x, (i >= 0) ? ("+") : (""), i);
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tR%u=MEM[0x%08X]=0x%02X\n", R[29], instrucao, z, endereco, R[z]);
            break;

            // l16
            case 0b011001:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = (R[x] + i) << 1;
                 
                // Leitura normal da memória

                R[z] = leDaCacheD(MEM8, &cacheD, endereco, output);

                // Formatacao da instrucao
                sprintf(instrucao, "l16 r%u,[r%u%s%i]", z, x, (i >= 0) ? ("+") : (""), i);
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tR%u=MEM[0x%08X]=0x%04X\n", R[29], instrucao, z, endereco, R[z]);
            break;

            // l32
            case 0b011010:
                // Obtendo operandos
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = (R[x] + i) << 2;

                // Verifica se o endereço é o do watchdog
                if (endereco == 0x80808080) {
                    // Leitura do registrador do watchdog
                    R[z] = ((wd.EN & 0x1) << 31) | (wd.COUNTER & 0x7FFFFFFF);
                } 
                else if (endereco == 0x80808880) {
                    R[z] = fpu.X;
                } 
                else if (endereco == 0x80808884) {
                    R[z] = fpu.Y;
                } 
                else if (endereco == 0x80808888) {
                    R[z] = fpu.Z;
                } 
                else if (endereco == 0x8080888C) {
                   R[z] = ((fpu.ST & 0x1) << 5) | (fpu.OP & 0x1F);
                } else {
                    // Leitura normal da memória

                    R[z] = leDaCacheD(MEM8, &cacheD, endereco, output);
                    
                }

                
                sprintf(instrucao, "l32 r%u,[r%u%s%i]", z, x, (i >= 0) ? ("+") : (""), i);
                
                fprintf(output, "0x%08X:\t%-25s\tR%u=MEM[0x%08X]=0x%08X\n", R[29], instrucao, z, endereco, R[z]);
            break;


            // s8
            case 0b011011:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = R[x] + i;

                // Verifica se o endereço é o do watchdog
                if (endereco == 0x80808080) {
                    wd.EN = (R[z] >> 31) & 0x1;      
                    wd.COUNTER = (R[z] & 0x7FFFFFFF) + 1;       
                    valorInstrucao = 0x80808080;
                }
                else if (endereco == 0x8888888B) {
                    terminal.OUT = R[z] & 0xFF;

                    // Armazena o caractere no buffer
                    if (terminal.bufferIndex < sizeof(terminal.buffer) - 1) {
                        terminal.buffer[terminal.bufferIndex++] = R[z] & 0xFF;
                        terminal.buffer[terminal.bufferIndex] = '\0';
                        
                    }
                }
                else if (endereco == 0x80808880) {
                    fpu.X = R[z];
                }
                else if (endereco == 0x80808884) {
                    fpu.Y = R[z];
                }
                else if (endereco == 0x80808888) {
                    fpu.Z = R[z];
                }
                else if (endereco == 0x8080888F) {
                    fpu.OP = R[z] & 0x1F;

                    executarFPU(MEM8, R, output);
                    fpu.fpuAtivado = 1;
                    
                    if (fpu.interrupcao == 2) {
                        fpu.fpuContador = fpu.ciclos2 + 1;
                    } 
                    else if (fpu.interrupcao == 3) {
                        fpu.fpuContador = fpu.ciclos3 + 1;
                    }
                    else if (fpu.interrupcao == 4) {
                        fpu.fpuContador = fpu.ciclos4 + 1;
                    }
                    
                } else {
                    // Escrita normal na memória
                    cacheD.typeMEMS = 8;
                    escreveNaCacheD(MEM8, &cacheD, endereco, R[z], output);

                    //MEM8[endereco] = R[z];
                }

                // Formatacao da instrucao
                sprintf(instrucao, "s8 [r%u%s%i],r%u", x, (i >= 0) ? ("+") : (""), i, z);
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]=R%u=0x%02X\n", R[29], instrucao, endereco, z, R[z] & 0xFF);
            break;

            // s16
            case 0b011100:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = (R[x] + i) << 1;                                                
                
                // Escrita normal na memória
                escreveNaCacheD(MEM8, &cacheD, endereco, R[z], output);             

                // Formatacao da instrucao
                sprintf(instrucao, "s16 [r%u%s%i],r%u", x, (i >= 0) ? ("+") : (""), i, z);
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]=R%u=0x%04X\n", R[29], instrucao, endereco, z, R[z]);
            break;

            // s32
            case 0b011101:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0xFFFF;

                // Calcula o endereço base
                endereco = (R[x] + i) << 2;
                
                    // Verifica se o endereço é o do watchdog
                if (endereco == 0x80808080) {
                    // Atualiza o watchdog: Bit 31 é o bit de habilitação, Bits 30 a 0 são o contador
                    wd.EN = (R[z] >> 31) & 0x1;      // Bit 31 é o bit de habilitação
                    wd.COUNTER = (R[z] & 0x7FFFFFFF) + 1;       // Bits 30 a 0 são o contador
                } 
                else if (endereco == 0x80808880) {
                    fpu.X = R[z];
                }
                else if (endereco == 0x80808884) {
                    fpu.Y = R[z];
                }
                else if (endereco == 0x80808888) {
                    fpu.Z = R[z];
                }
                else if (endereco == 0x8080888C) {
                    fpu.OP = R[z] & 0x1F;

                    executarFPU(MEM8, R, output);
                    fpu.fpuAtivado = 1;

                    if (fpu.interrupcao == 2) {
                        fpu.fpuContador = fpu.ciclos2 + 1;
                    } 
                    else if (fpu.interrupcao == 3) {
                        fpu.fpuContador = fpu.ciclos3 + 1;
                    }
                    else if (fpu.interrupcao == 4) {
                        fpu.fpuContador = fpu.ciclos4 + 1;
                    }

                } else {
                    // Escrita normal na memória

                    escreveNaCacheD(MEM8, &cacheD, endereco, R[z], output);

                }
        
                // Formatacao da instrucao
                sprintf(instrucao, "s32 [r%u%s%i],r%u", x, (i >= 0) ? ("+") : (""), i, z);
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tMEM[0x%08X]=R%u=0x%08X\n", R[29], instrucao, endereco, z, R[z]);
            break;

            case 0b011110: // CALLF
                
                x = (R[IR] & (0b11111 << 16)) >> 16;  // Extrai x (5 bits)
                int16_t i_imeddiate_callf = R[IR] & 0xFFFF;            // Extrai i (16 bits)
                
                oldPC = R[PC];

                // Extender o sinal do valor imediato i
                int32_t i_callf = (int32_t)(int16_t)i_imeddiate_callf; 

                escreveNaCacheD(MEM8, &cacheD, R[SP], (R[PC] + 4), output);
                

                R[SP] -= 4;   // Decrementa o SP em 4 após o armazenamento
                       
                // Calcular o novo valor do PC
                offset = (R[x] + i_callf) << 2; // Desloca o campo imediato i para obter o offset

                R[PC] = offset; // Define o novo valor de PC

                R[PC] -= 4;

                
                sprintf(instrucao, "call [r%d+%d]", x, i_callf); // Formata o registrador e o valor imediato

                
                fprintf(output, "0x%08X:\t%-25s\tPC=0x%08X,MEM[0x%08X]=0x%08X\n", 
                        oldPC,                       
                        instrucao,                       
                        R[PC] + 4,                           
                        R[SP] + 4,                       
                        oldPC + 4);                 
            break;

            case 0b011111: // RET
                oldPC = R[PC];
                
                R[SP] += 4;

                uint32_t return_adress = leDaCacheD(MEM8, &cacheD, R[SP], output);
                
                R[PC] = return_adress;
                R[PC] -= 4;

                
                sprintf(instrucao, "ret");

                
                fprintf(output, "0x%08X:\t%-25s\tPC=MEM[0x%08X]=0x%08X\n", 
                        oldPC,                      
                        instrucao,                      
                        R[SP],                          
                        R[PC] + 4);                   
            break;

            case 0b100000: // RETI
                oldPC = R[PC];

                R[SP] += 4;
                R[IPC] = leDaCacheD(MEM8, &cacheD, R[SP], output);
                

                R[SP] += 4;
                R[CR] = leDaCacheD(MEM8, &cacheD, R[SP], output);
                
                

                R[SP] += 4;
                R[PC] = leDaCacheD(MEM8, &cacheD, R[SP], output);
                
               
                R[PC] -= 4;

                sprintf(instrucao, "reti");
                // Formatacao de saida em tela
                fprintf(output, "0x%08X:\t%-25s\tIPC=MEM[0x%08X]=0x00000000,CR=MEM[0x%08X]=0x%08X,PC=MEM[0x%08X]=0x%08X\n", 
                    oldPC, instrucao, 
                    R[SP] - 8, // IPC
                    R[SP] - 4, R[CR],   // CR
                    R[SP], R[PC] + 4);  // PC
            break;

            case 0b100001:
                z = (R[IR] & (0b11111 << 21)) >> 21;
                x = (R[IR] & (0b11111 << 16)) >> 16;
                i = R[IR] & 0x0FFFF;
                uint8_t lastBit = i & 1;

                switch (lastBit) {

                    case 0b0: // CBR
                        R[z] &= ~(1 << x);

                        auxZ = registerNameLower(z);
                        auxZUpper = registerNameUpper(z);

                        sprintf(instrucao, "cbr %s[%u]", auxZ, x);
                        // Formatacao de saida em tela
                        fprintf(output, "0x%08X:\t%-25s\t%s=0x%08X\n", R[PC], instrucao, auxZUpper, R[z]);
                    break;

                    case 0b1: // SBR    
                        R[z] |= (1 << x);

                        auxZ = registerNameLower(z);
                        auxZUpper = registerNameUpper(z);

                        sprintf(instrucao, "sbr %s[%u]", auxZ, x);
                        // Formatacao de saida em tela
                        fprintf(output, "0x%08X:\t%-25s\t%s=0x%08X\n", R[PC], instrucao, auxZUpper, R[z]);
                    break;
                }
            break;

            default:
                fprintf(output,"[INVALID INSTRUCTION @ 0x%08X]\n", R[PC]);
                R[SR] |= (1 << IV);
                fprintf(output,"[SOFTWARE INTERRUPTION]\n");
                handleInterrupt(MEM8, R, output);
                R[CR] = (R[IR] & 0xFC000000) >> 26;
                R[IPC] = R[PC];
                R[PC] = 0x00000000;
            break;
        *executa = 0;
    }
}
