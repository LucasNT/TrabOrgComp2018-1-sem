//Turma 1 - Grupo 10
// Lucas Nobuyuki Takahashi     10295670
// Luiz Miguel di Mano Saraiva  10425420
// Marcelo Kiochi Hatanaka      10295645
// Rodrigo Mendes Andrade       10262721

// Lucas e Luiz implementaram a UC
// Marcelo e Rodrigo implementaram os outros modulos e a main

// compilar: gcc cpu_multi_code.c -o exe
// executar: ./exe code.bin

#include <stdio.h>
#include <stdlib.h>

// 'defines' para melhorar a legibilidade do codigo

// bits da variavel de sinais de controle
#define RegDst ((SinaisUC) & 0x3)
#define RegWrite ((SinaisUC >> 2) & 0x1)
#define ALUSrcA ((SinaisUC >> 3) & 0x1)
#define ALUSrcB ((SinaisUC >> 4) & 0x3)
#define ALUOp ((SinaisUC >> 6) & 0x3)
#define PCSource ((SinaisUC >> 8) & 0x3)
#define PCWriteCond ((SinaisUC >> 10) & 0x1)
#define PCWrite ((SinaisUC >> 11) & 0x1)
#define IorD ((SinaisUC >> 12) & 0x1)
#define MemRead ((SinaisUC >> 13) & 0x1)
#define MemWrite ((SinaisUC >> 14) & 0x1)
#define BNE ((SinaisUC >> 15) & 0x1)
#define IRWrite ((SinaisUC >> 16) & 0x1)
#define MemtoReg ((SinaisUC >> 17) & 0x3)

// bits separados do registrador de instrucao
#define IR_FNC_COD ((RegAux[IR].out) & 0x3f)    // bits do codigo de funcao IR[5...0]
#define IR_IMMED ((RegAux[IR].out) & 0xffff)    // bits do imediato IR[15...0]
#define IR_JMP ((RegAux[IR].out) & 0x3ffffff)   // bits do endereco do jump IR[25...0]
#define IR_RD ((RegAux[IR].out >> 11) & 0x1f)   // bits do RD IR[15...11]
#define IR_RT ((RegAux[IR].out >> 16) & 0x1f)   // bits do RT IR[20...16]
#define IR_RS ((RegAux[IR].out >> 21) & 0x1f)   // bits do RS IR[25...21]
#define IR_OP ((RegAux[IR].out >> 26) & 0x3f)   // bits do opcode IR[31...26]
// bits do shamt nao sao usados

// codigo de funcao para operacoes na ula para tipo_r
#define FNC_COD_ADD 32
#define FNC_COD_SUB 34
#define FNC_COD_SLT 42
#define FNC_COD_AND 36
#define FNC_COD_OR 37

// codigo de operacao da ula
#define ADD 0
#define SUB 1
#define SLT 2
#define AND 3
#define OR 4

// 'defines' para isolar cada bit do estado atual da maquina de estados da UC
#define S0 (s & 1)
#define S1 ((s & 2) >> 1)
#define S2 ((s & 4) >> 2)
#define S3 ((s & 8) >> 3)

// 'defines' para isolar os bits do opcode da instrucao
#define OP0 (opCode & 1)
#define OP1 ((opCode & 2) >> 1)
#define OP2 ((opCode & 4) >> 2)
#define OP3 ((opCode & 8) >> 3)
#define OP4 ((opCode & 16) >> 4)
#define OP5 ((opCode & 32) >> 5)

// tamanho da RAM
#define RAMTAM 256
// para visualizar a stack no final do programa, diminuir RAMTAM para 128

typedef struct r Register;

// indice para os registradores auxiliares no vetor
enum {
    PC,
    IR,
    MDR,
    A,
    B,
    ALUOut,
    QntReg
};

// indices para as mensagens de erro retornadas pelas unidade funcionais
enum{
    SUCCESS,
    UC_ERROR,
    MEM_ERROR,
    BCO_REG_ERROR,
    ALU_CTRL_ERROR
};

// definicao da struct dos registradores
struct r{
    int in, out;
    // 'in' eh o valor que sera salvo no registrador na subida do clock
    // 'out' eh o valor efetivamente salvo no registrador
    // na subida do clock, out recebe in
};

// variavel de sinais de controle
int SinaisUC;

// indice da ultima instrucao na RAM, para controlar a saida de erro
int lastInstructionPosition;

// vetor dos registradores auxiliares (PC, IR, MDR...)
Register RegAux[QntReg];

// vetor para o banco de registradores
Register BCO_REG[32];

// vetor para a RAM
unsigned char RAM[RAMTAM];

// variaveis que representam os barramentos de saida dos MUX
int MUX_IorD_out;
int MUX_MemtoReg_out;
int MUX_ALUSrcB_out;
int MUX_RegDst_out;
int MUX_ALUSrcA_out;
int MUX_PCSource_out;
int MUX_BNE_out;

// variaveis que representam os barramentos de saida dos modulos de shiftleft e sign extend
int SHT_JMP_out;
int SHT_BEQ_out;
int Sign_Extend_out;

// variaveis que representam o bit zero da ALU, a saida da ALU e a saida da unidade de controle da ALU
int ALU_Zero;
int ALU_out;
int ALU_CTRL_out;

// variavel que representa o resultado da operacao logica sobre os bits PCWrite, PCWriteCond e a saida do MUX_BNE
int PCWriteBit;

int UC(){
    static char s = 0; // variavel que armazena o estado atual
    static char n = 0; // variavel que armazena o proximo estado
    char opCode  = (IR_OP); // variavel que tem o opCode
    s = n; // passa para o proximo estado
    n = 0; // reseta o estado atual

    /*
     * o processo para definir o proximo estado basicamente define o bit mais significativo do proximo estado,
     * logo em seguida da um shift para esquerda e define o proximo bit, fazendo isso ate chegar no bit menos
     * significativo
     */

    // define o bit 3 , ou mais significativo do proximo estado
    n = (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //bne, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //beq, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //j, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & OP3 & !OP4 & !OP5) | // andi, estado 1
        ((!S0) & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // Addi, estado 2
        ((!S0) & !S1 & S2 & S3); // estado 12
    n = n << 1; // da um shift para esquerda

    //define o bit 2
    n = (n) |
        (S0 & S1 & !S2 & !S3) | // estado 3
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // SW, estado 2
        ((!S0) & S1 & S2 & !S3) | // estado 6
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & OP3 & !OP4 & !OP5) | // andi, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & !OP3 & !OP4 & !OP5); // TIPO-R, estado 1
    n = n << 1; // da um shift para esquerda


    //define o bit 1
    n = (n) |
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //bne, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & !OP3 & !OP4 & !OP5) |// TIPO-R, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // LW, estado 1
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // SW, estado 1
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // andi, estado 1
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // lw, estado 2
        ((!S0) & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // ADDI, estado 2
        ((!S0) & !S1 & S2 & S3) | // estado 12
        ((!S0) & S1 & S2 & !S3); // estado 6
    n = n << 1;

    //define o bit 0
    n = (n) |
        (S0 & !S1 & !S2 & !S3 & !OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //j, entrada 1
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal, entrada 1
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr, entrada 1
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // lw, entrada 2
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // Sw, entrada 2
        ((!S0) & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // ADDI, entrada 2
        ((!S0) & S1 & S2 & !S3) | //estado 6
        ((!S0) & !S1 & S2 & S3) | // estado 12
        ((!S0) & !S1 & !S2 & !S3); // estado 0

    SinaisUC = 0; // reseta a variavel de sinais da unidade de controle
    SinaisUC = (S3&S2&!S1&S0) | (S3&S2&S1&S0); //memtoreg1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3)&S2&!S1&!S0); //memtoreg0
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (!S3&!S2&!S1&!S0); //irwrite
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (S3&!S2&S1&!S0); //bne
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3)&S2&!S1&S0); //memwrite
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (!S3&!S2&S1&S0)); //memread
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (((!S3)&!S2&S1&S0) | ((!S3)&S2&!S1&S0)); //iord
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (S3&!S2&!S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&!S0) | (S3&S2&S1&S0)); //pcwrite
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0)); //pcwritecond
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&!S0) | (S3&S2&S1&S0)); //pcsrc1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&S1&S0) | (S3&S2&S1&!S0)); //pcsrc0
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (((!S3)&S2&S1&!S0) | (S3&S2&!S1&!S0)); //aluop1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&!S1&!S0)); //aluop0
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&S0) | (S3&S2&!S1&!S0) | (!S3&!S2&S1&!S0)); // alusrcB1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (!S3&!S2&!S1&S0)); //alusrcB0
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&S1&!S0) | ((!S3)&S2&S1&!S0) | (S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&!S1&!S0)); //alusrcA
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (((!S3)&S2&!S1&!S0) | ((!S3)&S2&S1&S0) | (S3&!S2&S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&S0)); //regwrite
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&S2&!S1&S0) ); //regdst1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3)&S2&S1&S0); //regdst0

    // verifica se o opCode é valido
    return !((!OP5 & !OP4 & !OP3 & !OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & OP2 & !OP1 & OP0) |
        (!OP5 & !OP4 & !OP3 & !OP2 & OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & !OP2 & OP1 & OP0) |
        ((!OP5) & OP4 & !OP3 & OP2 & !OP1 & !OP0) |
        ((!OP5) & OP4 & !OP3 & OP2 & !OP1 & OP0) |
        (OP5 & !OP4 & !OP3 & !OP2 & OP1 & OP0) |
        (OP5 & !OP4 & OP3 & !OP2 & OP1 & OP0) |
        (!OP5 & !OP4 & OP3 & !OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & OP3 & OP2 & !OP1 & !OP0));
}

// As funcoes que representam os MUX tem seus nomes a partir dos bits de sinal que os controla
// a funcao utiliza 'defines' para selecionar os bits da variavel de controle nos 'ifs'
// e salva sua saida na variavel de saida (nome da funcao + "_out")

// MUX do valor que indexa a memoria
void MUX_IorD(){
    if(IorD == 0x0){    // Seleciona PC
        MUX_IorD_out = RegAux[PC].out;
    } else {            // Seleciona saida da ALU
        MUX_IorD_out = RegAux[ALUOut].out;
    }
}

// MUX de Registrador de Destino do banco de registradores
void MUX_RegDst(){

    if(RegDst==0){      // Seleciona bits do RT da instrucao
        MUX_RegDst_out = IR_RT;
    }
    else if(RegDst==1){     // Seleciona bits do RD da instrucao
        MUX_RegDst_out = IR_RD;
    }
    else{                   // Seleciona registrador 31 (RA)
        MUX_RegDst_out = 31;
    }
}

// MUX de dado para escrever no banco de registradores
void MUX_MemtoReg(){
    if(MemtoReg == 0x0){            // Seleciona registrador auxiliar de saida da ALU

        MUX_MemtoReg_out = RegAux[ALUOut].out;

    } else if(MemtoReg == 0x1){     // Seleciona MDR
        MUX_MemtoReg_out = RegAux[MDR].out;
    } else {                        // Seleciona PC
        MUX_MemtoReg_out = RegAux[PC].out;
    }
}

// MUX de entrada A da ALU
void MUX_ALUSrcA(){
    if(ALUSrcA == 0){       // Seleciona PC
        MUX_ALUSrcA_out = RegAux[PC].out;
    }
    else{               // Seleciona registrador auxiliar A
        MUX_ALUSrcA_out = RegAux[A].out;
    }
}

// MUX de entrada B da ALU
void MUX_ALUSrcB(){

    if(ALUSrcB == 0){           // Seleciona registrador auxiliar B
        MUX_ALUSrcB_out = RegAux[B].out;
    } else if(ALUSrcB == 1){    // Seleciona valor 4
        MUX_ALUSrcB_out = 4;
    } else if(ALUSrcB == 2){    // Seleciona saida do modulo de Sign Extend
        MUX_ALUSrcB_out = Sign_Extend_out;
    } else {                    // Seleciona saida do modulo de shiftleft do BEQ

        MUX_ALUSrcB_out = SHT_BEQ_out;
    }

}

// MUX de valor de escrita no PC
void MUX_PCSource(){

    if(PCSource==0){            // Seleciona saida da ALU
        RegAux[PC].in = ALU_out;
    }
    else if(PCSource==1){       // Seleciona registrador auxiliar de saida da ALU
        RegAux[PC].in = RegAux[ALUOut].out;
    }
    else if(PCSource==2){       // Seleciona saida do modulo de shiftleft do JMP
        RegAux[PC].in = SHT_JMP_out;
    }
    else{                       // Seleciona registrador auxiliar A
        RegAux[PC].in = RegAux[A].out;
    }
}

// MUX para o bit Zero da ALU
void MUX_BNE(){
    if(BNE == 0){   // Seleciona o proprio bit
        MUX_BNE_out = ALU_Zero;
    } else{         // Seleciona o bit negado
        MUX_BNE_out = !ALU_Zero;
    }
}

// Modulo de extensao do sinal do valor imediato
void Sign_Extend(){
    unsigned int sign = IR_IMMED >> 15;  // salva bit mais significativo do imediato em uma variavel auxiliar
    if (sign == 1){         // se bit==1, salva no auxiliar todos os bits a partir do 16 como 1
        sign = 0xffff0000;
    }
    // Salva na saida o imediato com o sinal extendido
    Sign_Extend_out = IR_IMMED | sign;
}

// Modulos de shiftleft para BEQ e JMP
void SHT_BEQ(){
    SHT_BEQ_out = Sign_Extend_out << 2;
}
void SHT_JMP(){
    SHT_JMP_out = IR_JMP << 2;
}

// Unidade de Controle da ALU
int ALU_CTRL(){
    // Seleciona operacao da ALU, salvando o valor na variavel de saida
    if(ALUOp == 0){
        ALU_CTRL_out = ADD;
    }
    else if(ALUOp == 1){
        ALU_CTRL_out = SUB;
    }
    // Se for intrucao tipo_R, utiliza o codigo de funcao do IR
    else if(ALUOp == 2){
        if(IR_FNC_COD == FNC_COD_ADD){
            ALU_CTRL_out = ADD;
        }
        else if(IR_FNC_COD == FNC_COD_SUB){
            ALU_CTRL_out = SUB;
        }
        else if(IR_FNC_COD == FNC_COD_SLT){
            ALU_CTRL_out = SLT;
        }
        else if(IR_FNC_COD == FNC_COD_AND){
            ALU_CTRL_out = AND;
        }
        else if(IR_FNC_COD == FNC_COD_OR){
            ALU_CTRL_out = OR;
        }
        else{
            // Se codigo de funcao nao existe, retorna operacao invalida na ALU
            return ALU_CTRL_ERROR;
        }
    }
    else if(ALUOp == 3){
        ALU_CTRL_out = AND;
    }
    else{
        return ALU_CTRL_ERROR;
    }

    return SUCCESS;
}

// Unidade Logica e Aritmetica
void ALU(){
    // Seleciona operacao com base na saida da UC Secundaria
    // executa operacao com as saidas dos MUX e salva na variavel de saida
    if(ALU_CTRL_out == ADD){
        ALU_out = MUX_ALUSrcA_out + MUX_ALUSrcB_out;
    }
    else if(ALU_CTRL_out == SUB){
        ALU_out = MUX_ALUSrcA_out - MUX_ALUSrcB_out;
    }
    else if(ALU_CTRL_out == SLT){
        ALU_out = MUX_ALUSrcA_out < MUX_ALUSrcB_out;
    }
    else if(ALU_CTRL_out == AND){
        ALU_out = MUX_ALUSrcA_out & MUX_ALUSrcB_out;
    }
    else if(ALU_CTRL_out == OR){
        ALU_out = MUX_ALUSrcA_out | MUX_ALUSrcB_out;
    }

    // Atualiza o bit Zero com base na saida da operacao
    if(ALU_out == 0){
        ALU_Zero = 1;
    }
    else{
        ALU_Zero = 0;
    }

    // Salva saida no registrador auxiliar da ALU
    RegAux[ALUOut].in = ALU_out;
}

// Funcao para Escrita ou Leitura do banco de registradores
int Banco_Reg(){
    // Le do banco de registradores indexado pelo RS e RT da instrucao e salva
    // nos registradores auxiliares
    RegAux[A].in = BCO_REG[IR_RS].out;
    RegAux[B].in = BCO_REG[IR_RT].out;

    // Se o bit de sinal RegWrite estiver ativo
    if(RegWrite == 1){
        // Se registrador selecionado para escrita for invalida ($zero, $k0 ou $k1) retorna erro
        if(MUX_RegDst_out == 0 || MUX_RegDst_out == 26 || MUX_RegDst_out == 27){
            return BCO_REG_ERROR;
        }
        // Escreve no registrador indexado pela saida do MUX_RegDst
        // o valor selecionado no MUX_MemtoReg
        BCO_REG[MUX_RegDst_out].in = MUX_MemtoReg_out;
    }

    return SUCCESS;
}

// Funcao para Escrita ou Leitura da RAM
int Mem(){
    // Se posicao selecionada for invalida (negativa, maior que o tamanho da ram ou desalinhada), retorna erro
    if(MUX_IorD_out < 0 || MUX_IorD_out >= RAMTAM || MUX_IorD_out%4 != 0){
        return MEM_ERROR;
    }

    // aux aponta para a posicao da RAM selecionada, interpretando valor como palavra de 32 bits (int)
    int *aux = (int *) &RAM[MUX_IorD_out];
    // Se bit de controle MemRead estiver ativo, escreve em IR e MDR
    if(MemRead){
        RegAux[MDR].in = *aux;
        RegAux[IR].in = *aux;
    }

    // Se bit de controle memWrite estiver ativo, escreve na posicao selecionada o
    // valor do registrador auxiliar B
    if(MemWrite){
        // Se posicao selecionada for do segmento de texto, retorna erro
        if(MUX_IorD_out <= lastInstructionPosition)
            return MEM_ERROR;
        *aux = RegAux[B].out;
    }

    return SUCCESS;
}

// Modulo da operacao logica de PCWrite, PCWriteCond e a saida do MUX_BNE
void PCWriteFNC(){
    PCWriteBit = (MUX_BNE_out & PCWriteCond) | PCWrite;
}

// Funcao para atualizacao dos registradores, que dependem da subida do clock
// com base no sinais de controle que habilitam a escrita nos mesmos
void ClockUp(){
    if(PCWriteBit){
        RegAux[PC].out = RegAux[PC].in;
    }

    if(IRWrite){
        RegAux[IR].out = RegAux[IR].in;

    }

    if(RegWrite){
        BCO_REG[MUX_RegDst_out].out = BCO_REG[MUX_RegDst_out].in;
    }

    RegAux[MDR].out = RegAux[MDR].in;
    RegAux[A].out = RegAux[A].in;
    RegAux[B].out = RegAux[B].in;
    RegAux[ALUOut].out = RegAux[ALUOut].in;
}

int main(int argc, char const *argv[]){
    int i = 1;
    unsigned int inst;  // instrucao
    int erro = 0;
    int memPosition = 0;
    unsigned int *aux;  // auxiliar para encrever na RAM por palavra
    FILE *infile = NULL;

    if(argc != 2){
        printf("Numero invalido de argumentos. Execute: %s nome_arquivo_entrada\n" , argv[0]);
        return 0;
    }

    // Inicializa RAM, banco de registradores e registradores auxiliares com 0
    for(i=0; i<RAMTAM; i++){
        RAM[i] = 0;
    }
    for(i=0; i<32; i++){
        BCO_REG[i].in = 0;
        BCO_REG[i].out = 0;
    }
    for(i=0; i<QntReg; i++){
        RegAux[i].in = 0;
        RegAux[i].out = 0;
    }

    // Abre arquivo de entrada
    infile = fopen(argv[1], "r");

    if(infile==NULL){
        printf("Arquivo de entrada nao existe.\n");
        return 0;
    }

    // Le do arquivo de entrada e salva na RAM
    while(fscanf(infile, "%u", &inst) != EOF){
        // memPosition indexa posicao (byte) atual na RAM
        aux = (unsigned int *)&RAM[memPosition];
        // aux recebe endereco da posicao atual e interpreta como [int] para escrever palavra inteira
        *aux = inst;
        memPosition = memPosition + 4;
    }

    fclose(infile);

    // Salva posicao na RAM da ultima instrucao em variavel auxiliar global
    // para saber fim do segmento de texto
    lastInstructionPosition = memPosition - 4;

    BCO_REG[29].out = RAMTAM;   // Stack Pointer = ultima posicao de memoria + 1

    while(1){
        // Chama funcoes dos componentes da CPU por ordem de dependencia
        // Se uma unidade retorna erro, encerra a execucao do codigo
        erro = UC();
        if(erro != 0){ break; }

        MUX_IorD();

        erro = Mem();
        if(erro != 0){ break; }

        MUX_RegDst();
        MUX_MemtoReg();

        erro = Banco_Reg();
        if(erro != 0){ break; }

        Sign_Extend();
        SHT_BEQ();
        SHT_JMP();
        MUX_ALUSrcA();
        MUX_ALUSrcB();

        erro = ALU_CTRL();
        if(erro != 0){ break; }

        ALU();
        MUX_PCSource();
        MUX_BNE();
        PCWriteFNC();
        ClockUp();
    }

    // Imprime dados de saida
    printf("\nStatus da Saída: ");

    // Ao chegar ao fim do codigo de entrada, se nao houver nenhuma instrucao invalida, programa encontrara
    // uma instrucao de opcode 0 com codigo de funcao invalido (0).

    switch(erro){
        // Instrucao invalida eh detectada pela UC
        case UC_ERROR:
            printf("Termino devido a tentativa de execucao de instrucao invalida. Codigo de operacao = ");
            // Imprime codigo de operacao da instrucao lida
            for(i=0; i<6; i++){
                printf("%d", ((IR_OP) >> (5-i))&0x1);
            }
            printf(" (%d)", IR_OP);
            break;
        case MEM_ERROR:
            printf("Termino devido a acesso invalido de memoria. Posicao acessada = %d\n", MUX_IorD_out);
            break;
        case BCO_REG_ERROR:
            printf("Termino devido a acesso invalido ao banco de registradores. Numero de registrador acessado = %d\n", MUX_RegDst_out);
            break;
        case ALU_CTRL_ERROR:
            printf("Termino devido a operacao invalida da ULA. Codigo de Funcao = ");
            // Imprime codigo de funcao da instrucao lida
            for(i=0; i<6; i++){
                printf("%d", ((IR_FNC_COD) >> (5-i))&0x1);
            }
            printf(" (%d)", IR_FNC_COD);
            break;
    }

    printf("\n");

    // Imprime conteudo de registradores auxiliares
    printf("PC = %d\t", RegAux[PC].out);
    printf("IR = %u\t", RegAux[IR].out);
    printf("MDR = %u\n", RegAux[MDR].out);
    printf("A = %d\t", RegAux[A].out);
    printf("B = %d\t", RegAux[B].out);
    printf("ALUOut = %d\n\n", RegAux[ALUOut].out);

    // Imprime bits da variavel de controle
    printf("Controle = [");
    printf("RegDst=%d", RegDst >> 1);
    printf("%d, ", (RegDst & 0x1));
    printf("RegWrite=%d, ", RegWrite);
    printf("ALUSrcA=%d, ", ALUSrcA);
    printf("ALUSrcB=%d", ALUSrcB >> 0x1);
    printf("%d, ", ALUSrcB & 0x1);
    printf("ALUOp=%d", ALUOp >> 1);
    printf("%d, ", ALUOp & 0x1);
    printf("PCSource=%d", PCSource >> 1);
    printf("%d, ", PCSource & 0x1);
    printf("PCWrite=%d,\n", PCWrite);
    printf("\tIorD=%d, ", IorD);
    printf("MemRead=%d, ", MemRead);
    printf("MemWrite=%d, ", MemWrite);
    printf("BNE=%d, ", BNE);
    printf("IRWrite=%d, ", IRWrite);
    printf("MemtoReg=%d", MemtoReg >> 1);
    printf("%d] ", MemtoReg & 0x1);
    printf("(%d)", SinaisUC);

    printf("\n\n");

    // Imprime 32 primeiras posicoes de memoria
    printf("Memoria (enderecos a byte):\n");
    for(i=0; i<32; i=i+4){
        printf("RAM[%d] = %u\tRAM[%d] = %u\tRAM[%d] = %u\tRAM[%d] = %u",
            i, *((int *)&RAM[i]), i+32, *((int *)&RAM[i+32]), i+64, *((int *)&RAM[i+64]), i+96, *((int *)&RAM[i+96]));
        printf("\n");
    }

    printf("\n");

    printf("Banco de registradores:\n");
    // Imprime conteudo do banco de registradores
    printf("BCO_REG[%d](zero) = %d\tBCO_REG[%d](t0) = %d\tBCO_REG[%d](s0) = %d\tBCO_REG[%d](t8) = %d",
        0, BCO_REG[0].out, 8, BCO_REG[8].out, 16, BCO_REG[16].out, 24, BCO_REG[24].out);
    printf("\n");
    printf("BCO_REG[%d](at) = %d\tBCO_REG[%d](t1) = %d\tBCO_REG[%d](s1) = %d\tBCO_REG[%d](t9) = %d",
        1, BCO_REG[1].out, 9, BCO_REG[9].out, 17, BCO_REG[17].out, 25, BCO_REG[25].out);
    printf("\n");
    printf("BCO_REG[%d](v0) = %d\tBCO_REG[%d](t2) = %d\tBCO_REG[%d](s2) = %d\tBCO_REG[%d](k0) = %d",
        2, BCO_REG[2].out, 10, BCO_REG[10].out, 18, BCO_REG[18].out, 26, BCO_REG[26].out);
    printf("\n");
    printf("BCO_REG[%d](v1) = %d\tBCO_REG[%d](t3) = %d\tBCO_REG[%d](s3) = %d\tBCO_REG[%d](k1) = %d",
        3, BCO_REG[3].out, 11, BCO_REG[11].out, 19, BCO_REG[19].out, 27, BCO_REG[27].out);
    printf("\n");
    printf("BCO_REG[%d](a0) = %d\tBCO_REG[%d](t4) = %d\tBCO_REG[%d](s4) = %d\tBCO_REG[%d](gp) = %d",
        4, BCO_REG[4].out, 12, BCO_REG[12].out, 20, BCO_REG[20].out, 28, BCO_REG[28].out);
    printf("\n");
    printf("BCO_REG[%d](a1) = %d\tBCO_REG[%d](t5) = %d\tBCO_REG[%d](s5) = %d\tBCO_REG[%d](sp) = %d",
        5, BCO_REG[5].out, 13, BCO_REG[13].out, 21, BCO_REG[21].out, 29, BCO_REG[29].out);
    printf("\n");
    printf("BCO_REG[%d](a2) = %d\tBCO_REG[%d](t6) = %d\tBCO_REG[%d](s6) = %d\tBCO_REG[%d](fp) = %d",
        6, BCO_REG[6].out, 14, BCO_REG[14].out, 22, BCO_REG[22].out, 30, BCO_REG[30].out);
    printf("\n");
    printf("BCO_REG[%d](a3) = %d\tBCO_REG[%d](t7) = %d\tBCO_REG[%d](s7) = %d\tBCO_REG[%d](ra) = %d",
        7, BCO_REG[7].out, 15, BCO_REG[15].out, 23, BCO_REG[23].out, 31, BCO_REG[31].out);
    printf("\n\n");

    return 0;
}
