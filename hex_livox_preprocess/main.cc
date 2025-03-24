/****************************************************************
 * Copyright 2024 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2024-05-12
 ****************************************************************/

#include "hex_livox_preprocess/data_interface/data_interface.h"
#include "hex_livox_preprocess/hex_livox_preprocess.h"

using hex::preprocess::DataInterface;
using hex::preprocess::HexLivoxPreprocessed;
using hex::preprocess::HexLogLevel;

const char kNodeName[] = "hex_livox_preprocess";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexLivoxPreprocessed& hex_livox_preprocess =
      HexLivoxPreprocessed::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (hex_livox_preprocess.Init()) {
        data_interface.Log(HexLogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (hex_livox_preprocess.Work()) {
        // data_interface.Log(HexLogLevel::kInfo, "%s : Work Succeded",
        // kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(HexLogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetSingleton();
  data_interface.Init(argc, argv, kNodeName, 2.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();
  return 0;
}
