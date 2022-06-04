from Data import Read_data
from gurobipy import *
import copy
class Solve:
    def __init__(self):
        #input
        self.multiplier=10
        data=Read_data(self.multiplier)
        self.node_list, self.link_list, self.OD_pair_list, \
        self.g_number_of_links, self.g_number_of_nodes, self.g_number_of_ODs, self.candidate_wireless_charging_lanes, \
        self.candidate_wireless_charging_link_id, self.g_number_of_wireless_charging_lanes, self.SP_list = data.read_links()

        self.electric_quantity=6 # battery capacity

        self.charging_quantity_per_time=1

        # self.construction_budget=0            # number of constructed charging lanes?
        self.construction_cost_per_distance=10  #The total construction can be set 1?

        self.ratio_time_budget=1.4
        self.column_generation_time=1000

        #DP parameter
        self.Best_K_Size=3000

        #Columns
        # self.solutions_of_KS_subproblem=[]
        self.space_of_routing_subproblem = [[] for i in range(self.g_number_of_ODs)]  # state-space-time path
        self.time_of_routing_subproblem = [[] for i in range(self.g_number_of_ODs)]  # state-space-time path
        self.state_of_routing_subproblem = [[] for i in range(self.g_number_of_ODs)]  # state-space-time path
        self.charging_wireless_traveling_flag = [[] for i in range(self.g_number_of_ODs)]
        # self.primal_cost_of_routing_subproblem=[[] for i in range(self.g_number_of_ODs)]


        #output
        self.reduced_cost_of_routing_subproblem = [[] for i in range(self.g_number_of_ODs)]
        # self.reduced_cost_of_KS_subproblem = []
        self.optimal_space_seq = []
        self.optimal_time_seq = []
        self.optimal_state_seq = []
        self.optimal_objective_value=[]
        self.optimal_construction_result = []
        self.record_multiplier_miu = []  # dual variables
        self.global_LB = None
        self.global_UB = 10000
        self.obj_of_RMP = []
        # self.accessible_Flag=[]
        self.dual_variable_for_flow_balance=[]

        # self.g_solving_routing_subproblem(0, 1)


    def g_solving_FCCSLP_by_CG(self):
        print("Solving...")

#1. initialize the dual variable
        self.pi_list_for_flow_balance=[10]*self.g_number_of_ODs
        for i in range(self.column_generation_time):
            Flag=0
            print("iteration_{}".format(i))

# 3. column generation process
            #Pricing subproblems
            # solve the routing subproblems and add columns
            for od_index in range(self.g_number_of_ODs):
                OD_pair = self.OD_pair_list[od_index]
                self.g_solving_routing_subproblem(od_index,1)

                self.space_of_routing_subproblem[od_index].append(self.g_ending_state_vector.state_vector[0].m_visit_node_seq)
                self.time_of_routing_subproblem[od_index].append(self.g_ending_state_vector.state_vector[0].m_visit_time_seq)
                self.state_of_routing_subproblem[od_index].append(self.g_ending_state_vector.state_vector[0].m_visit_state_seq)
                self.charging_wireless_traveling_flag[od_index].append(self.g_ending_state_vector.state_vector[0].m_visit_charging_lanes)
                # primal_cost_for_OD_k = self.g_ending_state_vector.state_vector[0].Primal_label_cost
                # self.primal_cost_of_routing_subproblem[od_index].append(primal_cost_for_OD_k)

                LR_cost=self.g_ending_state_vector.state_vector[0].Label_cost_for_LR
                Reduced_cost_for_OD_k=LR_cost-self.pi_list_for_flow_balance[od_index]
                self.reduced_cost_of_routing_subproblem[od_index].append(Reduced_cost_for_OD_k)

                if Reduced_cost_for_OD_k < 0:
                    print(Reduced_cost_for_OD_k)
                    Flag=1

            # RMP
            self.record_multiplier_miu.append([])

            obj_of_RMP, self.pi_list_for_flow_balance = self.g_solving_RMP_by_LP(i)
            self.obj_of_RMP.append(obj_of_RMP)
            self.dual_variable_for_flow_balance.append(copy.copy(self.pi_list_for_flow_balance))


            if Flag == 0:
                self.global_LB = obj_of_RMP
                break
        # step 2: Generate an upper bound and output the optimal solution
        obj_of_RMP = self.g_solving_RMP_by_IP()
        self.global_UB = obj_of_RMP

        # if self.global_UB > obj_of_RMP:
        #     self.global_UB = obj_of_RMP
        print("Lb:{}".format(self.global_LB))
        print("Ub:{}".format(self.global_UB))



    def g_solving_RMP_by_LP(self, i):
        self.LP = Model("path_based_model")
        self.LP.setParam('OutputFlag', 0)
        # variables y_i_j
        for a in range(self.g_number_of_wireless_charging_lanes):
            charging_segment = self.candidate_wireless_charging_lanes[a]
            cost = charging_segment.travel_time * self.construction_cost_per_distance
            self.LP.addVar(lb=0, ub=1, obj=cost, vtype=GRB.CONTINUOUS, name="y_{}".format(a))

        # variable X_m_k
        for k in range(self.g_number_of_ODs):
            columns_for_OD_k = self.space_of_routing_subproblem[k]
            for m in range(len(columns_for_OD_k)):
                self.LP.addVar(lb=0, ub=1, obj=0, vtype=GRB.CONTINUOUS, name="x_{}_{}".format(k, m))
        self.LP.update()

        # constraint I: we should select a column for each OD pair
        for k in range(self.g_number_of_ODs):
            columns_for_OD_k = self.space_of_routing_subproblem[k]
            expr = LinExpr()
            for m in range(len(columns_for_OD_k)):
                name = self.LP.getVarByName(name="x_{}_{}".format(k, m))
                expr.addTerms(1, name)
            self.LP.addConstr(expr, GRB.EQUAL, 1, name="flow balance for OD-{}".format(k))

        # constraints II: coupling constraints
        for k in range(self.g_number_of_ODs):
            candidate_link_flag_for_OD_k = self.charging_wireless_traveling_flag[k]
            for link_id in range(self.g_number_of_wireless_charging_lanes):
                expr = LinExpr()
                # x
                for m in range(len(candidate_link_flag_for_OD_k)):
                    value = candidate_link_flag_for_OD_k[m][link_id]
                    name = self.LP.getVarByName("x_{}_{}".format(k, m))
                    expr.addTerms(value, name)
                # y
                value = -1
                name = self.LP.getVarByName(name="y_{}".format(link_id))
                expr.addTerms(value, name)

                self.LP.addConstr(expr, GRB.LESS_EQUAL, 0, name="Coupling_{}_{}".format(link_id, k))

        self.LP.optimize()
        self.LP.write("RMP.lp")
        obj_of_RMP = self.LP.objval

        #obtain and update the dual variables
        pi_list_for_flow_balance = []
        for k in range(self.g_number_of_ODs):
            constr = self.LP.getConstrByName("flow balance for OD-{}".format(k))

            pi_list_for_flow_balance.append(1 * constr.Pi)

        for link_id in range(self.g_number_of_wireless_charging_lanes):
            multiplier_link = []
            for k in range(self.g_number_of_ODs):
                constr = self.LP.getConstrByName("Coupling_{}_{}".format(link_id, k))
                Pi = -1 * constr.Pi  # >0
                # Pi = abs(constr.Pi)  # >0
                multiplier_link.append(Pi)
            self.record_multiplier_miu[i].append(copy.copy(multiplier_link))
            self.candidate_wireless_charging_lanes[link_id].base_profit_for_lagrangian=multiplier_link

        return obj_of_RMP, pi_list_for_flow_balance

    def g_solving_RMP_by_IP(self):
        self.LP = Model("path_based_model")
        self.LP.setParam('OutputFlag', 0)
        # variables y_i_j
        for a in range(self.g_number_of_wireless_charging_lanes):
            charging_segment = self.candidate_wireless_charging_lanes[a]
            cost = charging_segment.travel_time * self.construction_cost_per_distance
            self.LP.addVar(lb=0, ub=1, obj=cost, vtype=GRB.BINARY, name="y_{}".format(a))

        # variable X_m_k
        for k in range(self.g_number_of_ODs):
            columns_for_OD_k = self.space_of_routing_subproblem[k]
            for m in range(len(columns_for_OD_k)):
                self.LP.addVar(lb=0, ub=1, obj=0, vtype=GRB.BINARY, name="x_{}_{}".format(k, m))
        self.LP.update()

        # constraint I: we should select a column for each OD pair
        for k in range(self.g_number_of_ODs):
            columns_for_OD_k = self.space_of_routing_subproblem[k]
            expr = LinExpr()
            for m in range(len(columns_for_OD_k)):
                name = self.LP.getVarByName(name="x_{}_{}".format(k, m))
                expr.addTerms(1, name)
            self.LP.addConstr(expr, GRB.EQUAL, 1, name="flow balance for OD-{}".format(k))

        # constraints II: coupling constraints
        for k in range(self.g_number_of_ODs):
            candidate_link_flag_for_OD_k = self.charging_wireless_traveling_flag[k]
            for link_id in range(self.g_number_of_wireless_charging_lanes):
                expr = LinExpr()
                # x
                for m in range(len(candidate_link_flag_for_OD_k)):
                    value = candidate_link_flag_for_OD_k[m][link_id]
                    name = self.LP.getVarByName("x_{}_{}".format(k, m))
                    expr.addTerms(value, name)
                # y
                value = -1
                name = self.LP.getVarByName(name="y_{}".format(link_id))
                expr.addTerms(value, name)

                self.LP.addConstr(expr, GRB.LESS_EQUAL, 0, name="Coupling_{}_{}".format(link_id, k))

        self.LP.optimize()
        self.LP.write("RMP.lp")
        obj_of_RMP = self.LP.objval
        solution=self.LP.getVars()
        # return obj_of_RMP

        # recover the optimal solution
        index=self.g_number_of_wireless_charging_lanes
        for k in range(self.g_number_of_ODs):
            columns_for_OD_k=self.space_of_routing_subproblem[k]
            for m in range(len(columns_for_OD_k)):
                if round(solution[index].x)==1:
                    self.optimal_space_seq.append(copy.copy(self.space_of_routing_subproblem[k][m]))
                    self.optimal_time_seq.append(copy.copy(self.time_of_routing_subproblem[k][m]))
                    self.optimal_state_seq.append(copy.copy(self.state_of_routing_subproblem[k][m]))
                    # self.optimal_objective_value.append(copy.copy(self.primal_cost_of_routing_subproblem[k][m]))
                index+=1

        self.optimal_construction_result=[]
        for link_id in range(self.g_number_of_wireless_charging_lanes):
            if round(solution[link_id].x) == 1:
                self.optimal_construction_result.append(1)
            else:
                self.optimal_construction_result.append(0)
        return obj_of_RMP

    def g_solving_routing_subproblem(self,od_index,flag):#core
        """
       flag=1:LR cost
       flag=2:primal cost
        """
        self.time_budget=int(self.SP_list[od_index]*self.ratio_time_budget)
        self.g_ending_state_vector = []
        self.g_time_dependent_state_vector = []
        for t in range(self.time_budget + 1):
            self.g_time_dependent_state_vector.append([None] * self.g_number_of_nodes)

        ending_flag=0
        OD_pair=self.OD_pair_list[od_index]

        #Initialization
        for t in range(0,self.time_budget+1):
            for n in range(self.g_number_of_nodes):
                self.g_time_dependent_state_vector[t][n]=C_time_indexed_state_vector()
                self.g_time_dependent_state_vector[t][n].Reset()
                self.g_time_dependent_state_vector[t][n].current_time=t
                self.g_time_dependent_state_vector[t][n].current_node=n
        self.g_ending_state_vector=C_time_indexed_state_vector()

        #new element 1: for the origin node
        element=CVSState(self.g_number_of_wireless_charging_lanes)
        element.current_node=OD_pair[0]
        element.current_time=0
        element.current_state=self.electric_quantity
        element.m_visit_node_seq.append(OD_pair[0])
        element.m_visit_time_seq.append(0)
        element.m_visit_state_seq.append(self.electric_quantity)
        self.g_time_dependent_state_vector[0][OD_pair[0]].update_state(element,flag)

        #new element 2: for the ending node
        # new_element = CVSState(self.g_number_of_wireless_charging_lanes)
        # new_element.my_copy(element)
        # new_element.current_node = OD_pair[1]
        # new_element.current_time = 1
        # new_element.current_state = self.electric_quantity
        # new_element.m_visit_node_seq.append(OD_pair[1])
        # new_element.m_visit_time_seq.append(1)
        # new_element.m_visit_state_seq.append(self.electric_quantity)
        # new_element.Primal_label_cost=1
        # new_element.Label_cost_for_LR=1
        # self.g_ending_state_vector.state_vector.append(new_element)
        # self.g_time_dependent_state_vector[1][OD_pair[1]].update_state(element)




        #DP gogogo
        for t in range(self.time_budget):
            if ending_flag==1:
                break
            for n in range(self.g_number_of_nodes):
                self.g_time_dependent_state_vector[t][n].Sort(flag)
                max_num=len(self.g_time_dependent_state_vector[t][n].state_vector)
                for index in range(min(self.Best_K_Size,max_num)):
                    pElement=self.g_time_dependent_state_vector[t][n].state_vector[index]
                    from_node_id=pElement.current_node
                    from_node=self.node_list[from_node_id]
                    #neigbors
                    for i in range(from_node.outbound_nodes_number):
                        to_node_id=from_node.outbound_nodes_list[i]
                        to_node=self.node_list[to_node_id]
                        link_to=from_node.outbound_links_list[i]
                        next_time=t+link_to.travel_time
                        #time resource
                        if next_time>self.time_budget:
                            continue

                        """
                        #destination node
                        if to_node_id==OD_pair[1]:
                            # electricty resource
                            electricity = pElement.current_state - link_to.consumed_electricity
                            if electricity < 0:
                                continue

                            new_element=CVSState(self.g_number_of_wireless_charging_lanes)
                            new_element.my_copy(pElement)
                            new_element.current_node=OD_pair[1]
                            new_element.current_time=next_time
                            new_element.current_state-=link_to.consumed_electricity
                            new_element.m_visit_node_seq.append(OD_pair[1])
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(new_element.current_state)
                            new_element.calculate_label_cost(0,0)
                            
                            
                            self.g_ending_state_vector.state_vector.append(new_element)

                            if flag==2 and new_element.Primal_label_cost==0:
                                ending_flag=1
                            if flag==1 and new_element.Label_cost_for_LR==0:
                                ending_flag=1

                            continue
                        """
                        # all links including charging links, but we do not charge
                        # electricty resource
                        electricity = pElement.current_state - link_to.consumed_electricity
                        if electricity < 0:
                            continue

                        new_element=CVSState(self.g_number_of_wireless_charging_lanes)
                        new_element.my_copy(pElement)
                        new_element.current_node = to_node_id
                        new_element.current_time = next_time
                        new_element.current_state -= link_to.consumed_electricity
                        new_element.m_visit_node_seq.append(to_node_id)
                        new_element.m_visit_time_seq.append(next_time)
                        new_element.m_visit_state_seq.append(new_element.current_state)
                        new_element.calculate_label_cost(0, 0)
                        self.g_time_dependent_state_vector[next_time][to_node_id].update_state(new_element,flag)

                        if to_node_id==OD_pair[1]:
                            self.g_ending_state_vector.state_vector.append(new_element)

                            if flag == 2 and new_element.Primal_label_cost == 0:
                                ending_flag = 1
                            if flag == 1 and new_element.Label_cost_for_LR == 0:
                                ending_flag = 1

                        #charging link
                        # if link_to.link_type!=0:
                        if link_to.wireless_charging_lane_flag!=0:
                            # electricty resource
                            current_electric_quantity = pElement.current_state - link_to.consumed_electricity + self.charging_quantity_per_time * link_to.travel_time
                            if current_electric_quantity < 0:
                                continue

                            charging_lane_index=link_to.wireless_charging_lane_flag #from 1
                            # if not constructed and the DP-2
                            if flag==2 and link_to.construction_Flag==0:
                                continue #not constructed

                            if new_element.m_visit_charging_lanes[charging_lane_index-1]==1:
                                continue #has been used

                            new_element=CVSState(self.g_number_of_wireless_charging_lanes)
                            new_element.my_copy(pElement)
                            new_element.current_node = to_node_id
                            new_element.current_time = next_time


                            new_element.current_state = min(self.electric_quantity,current_electric_quantity)
                            new_element.m_visit_charging_lanes[charging_lane_index-1]=1

                            # new_element.m_visit_charging_nodes.append(link_to.link_type)
                            new_element.m_visit_node_seq.append(to_node_id)
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(new_element.current_state)
                            #find the multiplier
                            multiplier=link_to.base_profit_for_lagrangian[od_index]
                            new_element.calculate_label_cost(0,multiplier)
                            self.g_time_dependent_state_vector[next_time][to_node_id].update_state(new_element,flag)

                            if to_node_id==OD_pair[1]:
                                self.g_ending_state_vector.state_vector.append(new_element)

                                if flag == 2 and new_element.Primal_label_cost == 0:
                                    ending_flag = 1
                                if flag == 1 and new_element.Label_cost_for_LR == 0:
                                    ending_flag = 1


        self.g_ending_state_vector.Sort(flag)

    def output_results(self,spend_time):
        with open("result.txt","w") as fl:
            fl.write("Global_LB: {}\n".format(self.global_LB))
            fl.write("Global_UB: {}\n".format(self.global_UB))
            if self.global_UB!=0:
                gap=(self.global_UB-self.global_LB)/self.global_UB
            else:
                gap=0
            fl.write("Gap: {}\n".format(gap))
            fl.write("Time: {} seconds\n".format(spend_time))
            fl.write("Construction: {}\n".format(self.optimal_construction_result))

        with open("Optimal_routing_solution.csv","w") as fl:
            fl.write("od_index,od_pair,space,time,state\n")
            for od_index in range(self.g_number_of_ODs):
                od_pair=self.OD_pair_list[od_index]
                space=self.optimal_space_seq[od_index]
                time=self.optimal_time_seq[od_index]
                state=self.optimal_state_seq[od_index]
                # value=self.optimal_objective_value[od_index]
                # if value==1:
                #     Accessible_Flag=0
                # if value==0:
                #     Accessible_Flag=1
                fl.write(str(od_index)+","+str(od_pair)+","+","+str(space)+","+str(time)+","+str(state)+"\n")

        with open("Constructed_segments.csv","w") as fl:
            fl.write("origin,destination\n")
            for index in range(self.g_number_of_wireless_charging_lanes):
                if self.optimal_construction_result[index]==1:
                    segment=self.candidate_wireless_charging_lanes[index]

                    O=segment.from_node_id+1
                    D=segment.to_node_id+1
                    fl.write(str(O)+","+str(D)+"\n")


        with open("reduced_costs.csv","w") as fl:
            fl.write("iteration,")
            for od_index in range(self.g_number_of_ODs):
                fl.write("od_{},".format(od_index+1))
            fl.write("\n")

            for i in range(len(self.reduced_cost_of_routing_subproblem[0])):
                fl.write(str(i+1)+",")
                for od_index in range(self.g_number_of_ODs):
                    reduced_cost=self.reduced_cost_of_routing_subproblem[od_index][i]
                    fl.write(str(reduced_cost)+",")
                fl.write("\n")
                # fl.write(str(self.reduced_cost_of_KS_subproblem[i])+"\n")

        with open("dual_variable_for_OD_pairs.csv","w") as fl:
            fl.write("iteration,")
            for od_index in range(self.g_number_of_ODs):
                fl.write("od_{},".format(od_index + 1))
            fl.write("\n")


            for i in range(len(self.dual_variable_for_flow_balance)):
                fl.write(str(i + 1) + ",")
                for od_index in range(self.g_number_of_ODs):
                    dual_variable = self.dual_variable_for_flow_balance[i][od_index]
                    fl.write(str(dual_variable) + ",")
                fl.write("\n")



        with open("output_dual_variables.csv", "w") as fl:
            fl.write("iteration,")
            for k in range(self.g_number_of_ODs):
                for linkid in range(self.g_number_of_wireless_charging_lanes):
                    fl.write("{}_{},".format(k, linkid))
            fl.write("\n")

            for i in range(len(self.reduced_cost_of_routing_subproblem[0])):
                fl.write(str(i + 1) + ",")
                for k in range(self.g_number_of_ODs):
                    for linkid in range(self.g_number_of_wireless_charging_lanes):
                        multiplier = self.record_multiplier_miu[i][linkid][k]
                        fl.write(str(multiplier) + ",")
                fl.write("\n")

        with open("RMP_value.csv","w") as fl:
            fl.write("iteration,RMP_value\n")
            for index in range(len(self.obj_of_RMP)):
                fl.write(str(index+1)+","+str(self.obj_of_RMP[index])+"\n")



class C_time_indexed_state_vector:
    def __init__(self):
        self.current_time=0
        self.current_node=0
        self.state_vector=[]
        self.state_map=[]

    def Reset(self):
        self.current_time =0
        self.current_node =0
        self.state_vector =[]
        self.state_map =[]

    def m_find_state_index(self,string_key):
        if string_key in self.state_map:
            return self.state_map.index(string_key)
        else:
            return -1

    def update_state(self,element,flag):
        #LR cost：flag=1
        if flag==1:
            if self.state_vector==[]:
                self.state_vector.append(element)
            else:
            # dominance rule
                Flag=1 #if we add this state
                for state in self.state_vector:
                    # state>>new state, do not add the new state and break
                    if state.Label_cost_for_LR<=element.Label_cost_for_LR and state.current_state>=element.current_state:
                        Flag=0
                        break
                    # state<<new state,delete the exsiting state
                    if state.Label_cost_for_LR>=element.Label_cost_for_LR and state.current_state<=element.current_state:
                        self.state_vector.remove(state)
                # if we come to here, it means that the new state are not dominated by any existing state
                # we can add the new state
                if Flag==1:
                    self.state_vector.append(element)


            # string_key=element.generate_string_key()
            # state_index=self.m_find_state_index(string_key)
            #
            #
            # if state_index == -1:
            #     self.state_vector.append(element)
            #     self.state_map.append(string_key)
            # else:
            #     if element.Label_cost_for_LR<self.state_vector[state_index].Label_cost_for_LR:
            #         self.state_vector[state_index]=element


        #primal cost: only keep one state
        if flag==2:
            if self.state_vector==[]:
                self.state_vector.append(element)
            else:
                if element.current_state>self.state_vector[0].current_state:
                    self.state_vector[0]=element

### flag: generalized routing cost
    def Sort(self,flag):
        if flag==1:
            self.state_vector=sorted(self.state_vector,key=lambda x:x.Label_cost_for_LR)

        if flag==2:
            self.state_vector=sorted(self.state_vector,key=lambda x:x.current_state)


class CVSState:
    def __init__(self,charging_lane_num):
        self.current_node=None
        self.current_time=None
        self.current_state=None
        self.m_visit_node_seq=[]
        self.m_visit_time_seq = []
        self.m_visit_state_seq = []
        self.m_visit_charging_lanes=[0]*charging_lane_num
### flag: generalized routing cost
        self.Primal_label_cost=0
        self.Label_cost_for_LR=0

    def generate_string_key(self):
        str=self.current_state
        return str

    def my_copy(self,pElement):
        self.current_node =copy.deepcopy(pElement.current_node)
        self.current_time = copy.deepcopy(pElement.current_time)
        self.current_state = copy.deepcopy(pElement.current_state)
        self.m_visit_node_seq = copy.deepcopy(pElement.m_visit_node_seq)
        self.m_visit_time_seq = copy.deepcopy(pElement.m_visit_time_seq)
        self.m_visit_state_seq = copy.deepcopy(pElement.m_visit_state_seq)
        self.m_visit_charging_lanes = copy.deepcopy(pElement.m_visit_charging_lanes)
### flag: generalized routing cost
        self.Primal_label_cost = copy.deepcopy(pElement.Primal_label_cost)
        self.Label_cost_for_LR = copy.deepcopy(pElement.Label_cost_for_LR)

    def calculate_label_cost(self,primal_cost,multiplier):
        #except for the dummy link (primal-cost=1), other primal cost equal 0
        #the cost value
        self.Primal_label_cost+=primal_cost
        self.Label_cost_for_LR+=primal_cost+multiplier
### flag: generalized routing cost