#include <iostream>
#include <sstream>
#include <fstream>
#include <stack>
#include <queue>
#include <map>
#include <set>

/// Returns queue of tokens and map, describing the number of arguments of each met function
std::pair<std::queue<std::string>, std::map<std::string, size_t> >
ShuntingYardAlgorithm(const char* expression) {
  std::string last_token;
  std::stack<std::string> operator_stack;
  std::queue<std::string> output_queue;
  std::vector<std::pair<std::string, size_t> > function_call_stack;
  std::set<std::string> calculated;
  std::map<std::string, size_t> argument_number_matcher;
  for (size_t i = 0, balance = 0; expression[i] != '\0'; ++i) {
    if (expression[i] == ' ') continue;
    if (expression[i] >= '0' && expression[i] <= '9') {
      int number = 0;
      while (expression[i] >= '0' && expression[i] <= '9') {
        number = number * 10 + expression[i] - '0';
        i++;
      }
      i--;
      last_token = std::to_string(number);
      output_queue.push(last_token);
      continue;
    }
    if (expression[i] == '*') {
      while (!operator_stack.empty() && argument_number_matcher.count(operator_stack.top())) {
        output_queue.push(operator_stack.top());
        operator_stack.pop();
      }
      last_token = "*";
      operator_stack.push(last_token);
      continue;
    }
    if (expression[i] == '+') {
      while (!operator_stack.empty() &&
             (operator_stack.top() == "#" || operator_stack.top() == "*" ||
              operator_stack.top() == "-" || argument_number_matcher.count(operator_stack.top()))) {
        output_queue.push(operator_stack.top());
        operator_stack.pop();
      }
      last_token = "+";
      operator_stack.push(last_token);
      continue;
    }
    if (expression[i] == '-') {
      // unary minus
      if (last_token.empty() || last_token == "+" ||
          last_token == "-" || last_token == "*"  ||
          last_token == "(" || last_token == ",") {
        // we denote # as unary minus in operator stack
        while (!operator_stack.empty() &&
               (operator_stack.top() == "*" || argument_number_matcher.count(operator_stack.top()))) {
          output_queue.push(operator_stack.top());
          operator_stack.pop();
        }
        last_token = "#";
        operator_stack.push(last_token);
        continue;
      }
      // binary minus
      while (!operator_stack.empty() &&
             (operator_stack.top() == "#" || operator_stack.top() == "*" ||
              operator_stack.top() == "-" || argument_number_matcher.count(operator_stack.top()))) {
        output_queue.push(operator_stack.top());
        operator_stack.pop();
      }
      last_token = "-";
      operator_stack.push(last_token);
      continue;
    }
    if (expression[i] == '(') {
      balance++;
      last_token = "(";
      operator_stack.push(last_token);
      continue;
    }
    if (expression[i] == ',') {
      if (!calculated.count(function_call_stack.back().first)) {
        argument_number_matcher[function_call_stack.back().first]++;
      }
      while (!operator_stack.empty() && operator_stack.top() != "(") {
        output_queue.push(operator_stack.top());
        operator_stack.pop();
      }
      last_token = ",";
      continue;
    }
    if (expression[i] == ')') {
      balance--;
      if (!function_call_stack.empty() && balance == function_call_stack.back().second) {
        if (expression[i-1] != '(' && !calculated.count(function_call_stack.back().first)) { 
          argument_number_matcher[function_call_stack.back().first]++;
        }
        calculated.insert(std::string(function_call_stack.back().first));
        function_call_stack.pop_back();
      }
      while (!operator_stack.empty() && operator_stack.top() != "(") {
        output_queue.push(operator_stack.top());
        operator_stack.pop();
      }
      last_token = ")";
      operator_stack.pop();
      continue;
    }
    std::string name;
    for (; expression[i] >= 'a' && expression[i] <= 'z'; ++i) {
      name.push_back(expression[i]);
    }
    if (expression[i] == '(') {
      if (!calculated.count(name)) {
        argument_number_matcher[name] = 0;
      }
      function_call_stack.emplace_back(name, balance);
      operator_stack.push(name);
      operator_stack.emplace("(");
      last_token = "(";
      ++balance;
      continue;
    } else {
      output_queue.push(name);
      last_token = std::move(name);
      --i;
    }
  }
  while (!operator_stack.empty()) {
    output_queue.push(operator_stack.top());
    operator_stack.pop();
  }
  return std::make_pair(output_queue, argument_number_matcher);
}


typedef struct {
  const char * name;
  void       * pointer;
} symbol_t;

const unsigned int PC_REGISTER = 15;
const unsigned int LR_REGISTER = 14;
const unsigned int PUSH_R4_INSTRUCTION = 0xe92d0010;
const unsigned int BASE_POP_INSTRUCTION = 0xe8bd0000;
const unsigned int BASE_PUSH_INSTRUCTION = 0xe92d0000;
const unsigned int NEG_R0_INSTRUCTION = 0xe2600000;
const unsigned int ADD_R0_R0_R1_INSTRUCTION = 0xe0800001;
const unsigned int MUL_R0_R1_R0_INSTRUCTION = 0xe0000091;
const unsigned int SUB_R0_R1_R0_INSTRUCTION = 0xe0410000;
const unsigned int LDR_BASE_INSTRUCTION = 0xe5900000;
const unsigned int MOV_FROM_PC_BASE_INSTRUCTION = 0xe1a0000f;
const unsigned int BX_BASE_INSTRUCTION = 0xe12fff10;

void WriteInstruction(unsigned int instruction, unsigned int*& pointer) {
  *pointer = instruction;
  ++pointer;
}

unsigned int LdrInstruction(unsigned int from_register, unsigned int to_register) {
  return LDR_BASE_INSTRUCTION | (from_register << 16) | (to_register << 12);
}

unsigned int BxInstruction(unsigned int register_number) {
  return BX_BASE_INSTRUCTION | register_number;
}

unsigned int MovFromPCInstruction(unsigned int register_number) {
  return MOV_FROM_PC_BASE_INSTRUCTION | (register_number << 12);
}

void WriteConstantToRegister(unsigned int register_number, unsigned int data, unsigned int*& pointer) {
  WriteInstruction(LdrInstruction(PC_REGISTER, register_number), pointer);
  WriteInstruction(MovFromPCInstruction(PC_REGISTER), pointer);
  WriteInstruction(data, pointer);
}

unsigned int PopInstruction(bool pop_lr, unsigned int register_number) {
  unsigned int answer = BASE_POP_INSTRUCTION;
  unsigned int mask = 1u;
  for (size_t i = 0; i < register_number; i++, mask <<= 1) {
    answer |= mask;
  }
  if (pop_lr) answer |= (1u << 14);
  return answer;
}
unsigned int PopOneInstruction(unsigned int reg) {
  return BASE_POP_INSTRUCTION | (1u << reg);
}

unsigned int PushInstruction(bool push_lr, int register_number) {
  unsigned int answer = BASE_PUSH_INSTRUCTION;
  unsigned int mask = 1u;
  for (size_t i = 0; i < register_number; i++, mask <<= 1) {
    answer |= mask;
  }
  if (push_lr) answer |= (1u << 14);
  return answer;
}

extern "C" void
jit_compile_expression_to_arm(const char * expression,
                              const symbol_t * externs,
                              void * out_buffer) {
  auto cur_pointer = (unsigned int*) out_buffer;
  std::map<std::string, void*> extern_map;
  for (size_t i = 0; externs != nullptr && externs[i].pointer != 0; ++i) {
    extern_map[std::string(externs[i].name)] = externs[i].pointer;
  }
  auto [output_queue, numb_match] = ShuntingYardAlgorithm(expression);
  WriteInstruction(PushInstruction(true, 0), cur_pointer);
  WriteInstruction(PUSH_R4_INSTRUCTION, cur_pointer);
  while (!output_queue.empty()) {
    std::string u = std::move(output_queue.front());
    output_queue.pop();
    if (u == "#") {
      WriteInstruction(PopInstruction(false, 1), cur_pointer);
      WriteInstruction(NEG_R0_INSTRUCTION, cur_pointer); // neg r0, r0 (rsb)
      WriteInstruction(PushInstruction(false, 1), cur_pointer);
      continue;
    }
    if (u == "+") {
      WriteInstruction(PopInstruction(false, 2), cur_pointer);
      WriteInstruction(ADD_R0_R0_R1_INSTRUCTION, cur_pointer);
      WriteInstruction(PushInstruction(false, 1), cur_pointer);
      continue;

      // pop {r0, r1}
      // add r0, r0, r1
      // push {r0}
    }
    if (u == "*") {
      WriteInstruction(PopInstruction(false, 2), cur_pointer);
      WriteInstruction(MUL_R0_R1_R0_INSTRUCTION, cur_pointer);
      WriteInstruction(PushInstruction(false, 1), cur_pointer);
      continue;
      // pop {r0, r1}
      // mul r0, r1, r0
      // push {r0}
    }
    if (u == "-") {
      WriteInstruction(PopInstruction(false, 2), cur_pointer);
      WriteInstruction(SUB_R0_R1_R0_INSTRUCTION, cur_pointer);
      WriteInstruction(PushInstruction(false, 1), cur_pointer);

      continue;
      // pop {r0, r1}
      // sub r0, r1, r0
      // push {r0}
    }
    if (extern_map.count(u)) {
      if (numb_match.count(u)) {
        for (unsigned int j = numb_match[u]; j != 0; --j) {
          WriteInstruction(PopOneInstruction(j - 1), cur_pointer);
        }
        WriteConstantToRegister(4, reinterpret_cast<uintptr_t>(extern_map[u]), cur_pointer);
        WriteInstruction(MovFromPCInstruction(LR_REGISTER), cur_pointer); // mov lr, pc, that's how we return
        WriteInstruction(BxInstruction(4), cur_pointer); // bx r4
        WriteInstruction(PushInstruction(false, 1), cur_pointer);
        continue;
        
      } else {
        WriteConstantToRegister(0, reinterpret_cast<uintptr_t>(extern_map[u]), cur_pointer);
        WriteInstruction(LdrInstruction(0, 0), cur_pointer);
        WriteInstruction(PushInstruction(false, 1), cur_pointer);
        continue;
      }
    } else {
      std::stringstream string_stream(u);
      int res;
      auto res_ptr = (unsigned int*) &res;
      string_stream >> res;
      WriteConstantToRegister(0, *res_ptr, cur_pointer);
      WriteInstruction(PushInstruction(false, 1), cur_pointer);
      continue;
      // ldr r0, [pc]
      // mov pc, pc
      // 'number'
      // push {r0}
    }
  }
  WriteInstruction(PopInstruction(false, 1), cur_pointer);
  WriteInstruction(PopOneInstruction(4), cur_pointer);
  WriteInstruction(PopInstruction(true, 0), cur_pointer);
  WriteInstruction(BxInstruction(LR_REGISTER), cur_pointer);
}
