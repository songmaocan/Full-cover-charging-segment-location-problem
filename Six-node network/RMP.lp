\ Model path_based_model
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  10 y_0 + 20 y_1 + 30 y_2 + 10 y_3 + 20 y_4 + 30 y_5 + 10 y_6 + 20 y_7
   + 10 y_8 + 10 y_9 + 10 y_10 + 10 y_11 + 10 y_12
Subject To
 flow_balance_for_OD-0: x_0_0 + x_0_1 + x_0_2 + x_0_3 = 1
 flow_balance_for_OD-1: x_1_0 + x_1_1 + x_1_2 + x_1_3 = 1
 Coupling_0_0: - y_0 + x_0_0 + x_0_3 <= 0
 Coupling_1_0: - y_1 <= 0
 Coupling_2_0: - y_2 <= 0
 Coupling_3_0: - y_3 <= 0
 Coupling_4_0: - y_4 <= 0
 Coupling_5_0: - y_5 <= 0
 Coupling_6_0: - y_6 <= 0
 Coupling_7_0: - y_7 <= 0
 Coupling_8_0: - y_8 + x_0_1 <= 0
 Coupling_9_0: - y_9 <= 0
 Coupling_10_0: - y_10 + x_0_1 + x_0_2 <= 0
 Coupling_11_0: - y_11 <= 0
 Coupling_12_0: - y_12 <= 0
 Coupling_0_1: - y_0 <= 0
 Coupling_1_1: - y_1 <= 0
 Coupling_2_1: - y_2 <= 0
 Coupling_3_1: - y_3 + x_1_0 <= 0
 Coupling_4_1: - y_4 <= 0
 Coupling_5_1: - y_5 <= 0
 Coupling_6_1: - y_6 <= 0
 Coupling_7_1: - y_7 <= 0
 Coupling_8_1: - y_8 <= 0
 Coupling_9_1: - y_9 + x_1_1 <= 0
 Coupling_10_1: - y_10 + x_1_1 + x_1_2 + x_1_3 <= 0
 Coupling_11_1: - y_11 <= 0
 Coupling_12_1: - y_12 <= 0
Bounds
 y_0 <= 1
 y_1 <= 1
 y_2 <= 1
 y_3 <= 1
 y_4 <= 1
 y_5 <= 1
 y_6 <= 1
 y_7 <= 1
 y_8 <= 1
 y_9 <= 1
 y_10 <= 1
 y_11 <= 1
 y_12 <= 1
 x_0_0 <= 1
 x_0_1 <= 1
 x_0_2 <= 1
 x_0_3 <= 1
 x_1_0 <= 1
 x_1_1 <= 1
 x_1_2 <= 1
 x_1_3 <= 1
End
