#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains the result gatherer and write for CARLA scenarios.
It shall be used from the ScenarioManager only.
"""

from __future__ import print_function

import time
import datetime
import json
from tabulate import tabulate
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle, PageBreak
from reportlab.lib import colors


class ResultOutputProvider(object):

    """
    This module contains the _result gatherer and write for CARLA scenarios.
    It shall be used from the ScenarioManager only.
    """

    def __init__(self, data, result, stdout=True, filename=None, junitfile=None, jsonfile=None):
        """
        Setup all parameters
        - _data contains all scenario-related information
        - _result is overall pass/fail info
        - _stdout (True/False) is used to (de)activate terminal output
        - _filename is used to (de)activate file output in tabular form
        - _junit is used to (de)activate file output in junit form
        - _json is used to (de)activate file output in json form
        """
        self._data = data
        self._result = result
        self._stdout = stdout
        self._filename = filename
        self._junit = junitfile
        self._json = jsonfile


       ####---------------- Pass/Fail Flags--------------------####### 
        # self.collision_fail = False
        # self.instruction_fail = False
        # self.in_Lane_fail = False
        # self.redlight_fail = False
        # self.stop_sign_fail = False
        # self.speed_limit_fail = False
        # self.sidewalk_fail = False
        # self.wrong_lane_fail = False

        ###---------------------- Advice --------------------------####
        self.collision_advice = "No Comment Here"
        self.instruction_advice = "No Comment Here"
        self.in_Lane_advice = "No Comment Here"
        self.redlight_advice = "No Comment Here"
        self.stop_sign_advice = "No Comment Here"
        self.speed_limit_advice = "No Comment Here"
        self.sidewalk_advice = "No Comment Here"
        self.wrong_lane_advice = "No Comment Here"
        ###------------------------------------------------------------##########

        
        self._start_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                         time.localtime(self._data.start_system_time))
        self._end_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                       time.localtime(self._data.end_system_time))

    def write(self):
        """
        Public write function
        """

        if self._junit is not None:
            self._write_to_junit()
        if self._json is not None:
            self._write_to_reportjson()

        output = self.create_output_text()
        if self._filename is not None:
            with open(self._filename, 'w', encoding='utf-8') as fd:
                fd.write(output)
        if self._stdout:
            print(output)

    def create_output_text(self):
        """
        Creates the output message
        """
        output = "\n"
        output += " ======= Results of Scenario: {} ---- {} =======\n".format(
            self._data.scenario_tree.name, self._result)
        end_line_length = len(output) - 3
        output += "\n"

        # Lis of all the actors
        output += " > Ego vehicles:\n"
        for ego_vehicle in self._data.ego_vehicles:
            output += "{}; ".format(ego_vehicle)
        output += "\n\n"

        output += " > Other actors:\n"
        for actor in self._data.other_actors:
            output += "{}; ".format(actor)
        output += "\n\n"

        # Simulation part
        output += " > Simulation Information\n"

        system_time = round(self._data.scenario_duration_system, 2)
        game_time = round(self._data.scenario_duration_game, 2)
        ratio = round(self._data.scenario_duration_game / self._data.scenario_duration_system, 3)

        list_statistics = [["Start Time", "{}".format(self._start_time)]]
        list_statistics.extend([["End Time", "{}".format(self._end_time)]])
        list_statistics.extend([["Duration (System Time)", "{}s".format(system_time)]])
        list_statistics.extend([["Duration (Game Time)", "{}s".format(game_time)]])
        list_statistics.extend([["Ratio (System Time / Game Time)", "{}s".format(ratio)]])

        output += tabulate(list_statistics, tablefmt='fancy_grid')
        output += "\n\n"

        # Criteria part
        output += " > Criteria Information\n"
        header = ['Actor', 'Criterion', 'Result', 'Actual Value', 'Expected Value']
        list_statistics = [header]

        for criterion in self._data.scenario.get_criteria():
            name_string = criterion.name
            if criterion.optional:
                name_string += " (Opt.)"
            else:
                name_string += " (Req.)"

            actor = "{} (id={})".format(criterion.actor.type_id[8:], criterion.actor.id)
            criteria = name_string
            result = "FAILURE" if criterion.test_status == "RUNNING" else criterion.test_status
            actual_value = criterion.actual_value
            expected_value = criterion.expected_value_success

            list_statistics.extend([[actor, criteria, result, actual_value, expected_value]])

        # Timeout
        actor = ""
        criteria = "Timeout (Req.)"
        result = "SUCCESS" if self._data.scenario_duration_game < self._data.scenario.timeout else "FAILURE"
        actual_value = round(self._data.scenario_duration_game, 2)
        expected_value = round(self._data.scenario.timeout, 2)

        list_statistics.extend([[actor, criteria, result, actual_value, expected_value]])

        # Global and final output message
        list_statistics.extend([['', 'GLOBAL RESULT', self._result, '', '']])

        output += tabulate(list_statistics, tablefmt='fancy_grid')
        output += "\n"
        output += " " + "=" * end_line_length + "\n"

        return output

    def _write_to_reportjson(self):
        """
        Write a machine-readable report to JSON

        The resulting report has the following format:
        {
            criteria: [
                {
                    name: "CheckCollisions",
                    expected: "0",
                    actual: "2",
                    optional: false,
                    success: false
                }, ...
            ]
        }
        """
        json_list = []

        def result_dict(name, actor, optional, expected, actual, success):
            """
            Convenience function to convert its arguments into a JSON-ready dict
            :param name: Name of the test criterion
            :param actor: Actor ID as string
            :param optional: If the criterion is optional
            :param expected: The expected value of the criterion (eg 0 for collisions)
            :param actual: The actual value
            :param success: If the test was passed
            :return: A dict data structure that will be written to JSON
            """
            return {
                "name": name,
                "actor": actor,
                "optional": optional,
                "expected": expected,
                "actual": actual,
                "success": success,
            }

        for criterion in self._data.scenario.get_criteria():
            json_list.append(
                result_dict(
                    criterion.name,
                    "{}-{}".format(criterion.actor.type_id[8:], criterion.actor.id),
                    criterion.optional,
                    criterion.expected_value_success,
                    criterion.actual_value,
                    criterion.test_status in ["SUCCESS", "ACCEPTABLE"]
                )
            )

        # add one entry for duration
        timeout = self._data.scenario.timeout
        duration = self._data.scenario_duration_game
        json_list.append(
            result_dict(
                "Duration", "all", False, timeout, duration, duration <= timeout
            )
        )

        result_object = {
            "scenario": self._data.scenario_tree.name,
            "success": self._result in ["SUCCESS", "ACCEPTABLE"],
            "criteria": json_list
        }

        with open(self._json, "w", encoding='utf-8') as fp:
            json.dump(result_object, fp, indent=4)

    def _write_to_junit(self):
        """
        Writing to Junit XML
        """
        test_count = 0
        failure_count = 0
        for criterion in self._data.scenario.get_criteria():
            test_count += 1
            if criterion.test_status != "SUCCESS":
                failure_count += 1

        # handle timeout
        test_count += 1
        if self._data.scenario_duration_game >= self._data.scenario.timeout:
            failure_count += 1

        with open(self._junit, "w", encoding='utf-8') as junit_file:

            junit_file.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")

            test_suites_string = ("<testsuites tests=\"%d\" failures=\"%d\" disabled=\"0\" "
                                  "errors=\"0\" timestamp=\"%s\" time=\"%5.2f\" "
                                  "name=\"Simulation\" package=\"Scenarios\">\n" %
                                  (test_count,
                                   failure_count,
                                   self._start_time,
                                   self._data.scenario_duration_system))
            junit_file.write(test_suites_string)

            test_suite_string = ("  <testsuite name=\"%s\" tests=\"%d\" failures=\"%d\" "
                                 "disabled=\"0\" errors=\"0\" time=\"%5.2f\">\n" %
                                 (self._data.scenario_tree.name,
                                  test_count,
                                  failure_count,
                                  self._data.scenario_duration_system))
            junit_file.write(test_suite_string)

            for criterion in self._data.scenario.get_criteria():
                testcase_name = criterion.name + "_" + \
                    criterion.actor.type_id[8:] + "_" + str(criterion.actor.id)
                result_string = ("    <testcase name=\"{}\" status=\"run\" "
                                 "time=\"0\" classname=\"Scenarios.{}\">\n".format(
                                     testcase_name, self._data.scenario_tree.name))
                if criterion.test_status != "SUCCESS":
                    result_string += "      <failure message=\"{}\"  type=\"\"><![CDATA[\n".format(
                        criterion.name)
                    result_string += "  Actual:   {}\n".format(
                        criterion.actual_value)
                    result_string += "  Expected: {}\n".format(
                        criterion.expected_value_success)
                    result_string += "\n"
                    result_string += "  Exact Value: {} = {}]]></failure>\n".format(
                        criterion.name, criterion.actual_value)
                else:
                    result_string += "  Exact Value: {} = {}\n".format(
                        criterion.name, criterion.actual_value)
                result_string += "    </testcase>\n"
                junit_file.write(result_string)

            # Handle timeout separately
            result_string = ("    <testcase name=\"Duration\" status=\"run\" time=\"{}\" "
                             "classname=\"Scenarios.{}\">\n".format(
                                 self._data.scenario_duration_system,
                                 self._data.scenario_tree.name))
            if self._data.scenario_duration_game >= self._data.scenario.timeout:
                result_string += "      <failure message=\"{}\"  type=\"\"><![CDATA[\n".format(
                    "Duration")
                result_string += "  Actual:   {}\n".format(
                    self._data.scenario_duration_game)
                result_string += "  Expected: {}\n".format(
                    self._data.scenario.timeout)
                result_string += "\n"
                result_string += "  Exact Value: {} = {}]]></failure>\n".format(
                    "Duration", self._data.scenario_duration_game)
            else:
                result_string += "  Exact Value: {} = {}\n".format(
                    "Duration", self._data.scenario_duration_game)
            result_string += "    </testcase>\n"
            junit_file.write(result_string)

            junit_file.write("  </testsuite>\n")
            junit_file.write("</testsuites>\n")


###--------------------------------------Report Generation-----------------------------------------####################
    def generateReport(self):
        # # Get today's date, start time, and end time from the OS system

        game_time = round(self._data.scenario_duration_game, 2)


        list_stats = []

        today = datetime.date.today().strftime('%Y-%m-%d')
        end_time = datetime.datetime.now().replace(microsecond=0)

        for criterion in self._data.scenario.get_criteria():
            name_string = criterion.name
            if criterion.optional:
                name_string += " (Opt.)"
            else:
                name_string += " (Req.)"

            criteria = name_string
            result = "FAILURE" if criterion.test_status == "RUNNING" else criterion.test_status
            actual_value = criterion.actual_value

            list_stats.extend([[criteria, result, actual_value]])





        print(list_stats)

        cartype = "Dodge Charger"

    
#         # Define other data for the table 1
#         num_faults = 0


#         #################################### fail-variables#################################


        immediate_fail = 'No'


####------------------ Advice Given --------------------#####################


        if(list_stats[0][1]== 'FAILURE'): # collision
            self.collision_advice = "To reduce the risk of collisions, keep \na safe distance from other vehicles, be aware\n of your surroundings, and anticipate other drivers'\nactions. Obey traffic signals and speed limits,\n avoid distractions, and  never drive under \nthe influence. Prioritize safety at all times\n while driving. "
            immediate_fail = "Yes"
        else:
            self.collision_advice = "No Comment Here"

        if(list_stats[2][1]== 'FAILURE'):# Instruction
            self.instruction_advice = "Based on your driving data, it appears \n that you may have violated some instructions\n given by your driving instructor. We recommend \ncarefully reviewing the instructions you were \ngiven and practicing the appropriate driving\n techniques to ensure that you're following\n them correctly."
            immediate_fail = "Yes"
        else:
            self.instruction_advice = "No Comment Here"

        if( list_stats[3][1]== 'FAILURE'): # In Lane 
            self.in_Lane_advice = "Based on your driving data, it seems\n like you may need to focus on maintaining a consistent \nspeed and avoiding sudden lane changes, as \nthese can increase the likelihood of veering out \nof your lane. Try to anticipate traffic flow and\n adjust your speed and lane position\n accordingly to avoid sudden maneuvers."
        else:
            self.in_Lane_advice = "No Comment Here"

        if( list_stats[4][1]== 'FAILURE'): # RedLight 
            self.redlight_advice = "Based on your driving data,\n it is essential to always obey traffic signals\n and signs, especially when it comes to\n red lights, to avoid not only costly\n fines but also the risk of accidents that\n could harm yourself or other drivers."
            immediate_fail = "Yes"
        else:
            self.redlight_advice = "No Comment Here"

        if( list_stats[5][1]== 'FAILURE'): # Stop Sign 
            self.stop_sign_advice = "Based on your driving data, \nremember to always come to a complete stop\n at stop signs, approach them with caution, \nand yield to other vehicles or pedestrians\n to avoid violations and ensure the safety\n of yourself and others on the road."
        else:
            self.stop_sign_advice = "No Comment Here"

        if( list_stats[6][1]== 'FAILURE'): # Speed Limit 
            self.speed_limit_advice = "Based on your driving data, we recommend \n maintaining a speed of under 30 km/hr to\n improve your safety on the road. To achieve this, \n keep an eye on your speedometer and adjust \nyour speed accordingly. Remember, in real life, \ndriving slower can also help you save fuel,\n reduce wear and tear on your vehicle, \nand decrease emissions!"
        else:
            self.speed_limit_advice = "No Comment Here"


        if( list_stats[7][1]== 'FAILURE'): # Sidewalk
            self.sidewalk_advice = "Based on your driving data, it appears \nthat you may have gone over a sidewalk \nwhile driving. We recommend being more mindful of\n your vehicle's position on the road \nand avoiding any maneuvers that may\n cause you to leave the roadway."
            immediate_fail = "Yes"
        else:
            self.sidewalk_advice = "No Comment Here"        
        
        if( list_stats[8][1]== 'FAILURE'): # Wrong Lane 
            self.wrong_lane_advice = "Based on your driving data, it's\n important to stay in your designated lane, \nuse your turn signals, and make safe \nlane changes to avoid wrong lane violations\n and ensure the safety of yourself and other \ndrivers on the road."
            immediate_fail = "Yes"
        else:
            self.wrong_lane_advice = "No Comment Here"
          

#----------------- Immediate Fail -----------------

        if (immediate_fail == 'Yes'):
            self._result = 'Failed'
        

#--------------------num of faults fail---------------------------#

        num_of_faults = 0
        flag_for_inRoute = False
        flag_for_inLane = False

        num_of_faults = list_stats[0][2] + list_stats[4][2] + list_stats[5][2] + list_stats[6][2] + list_stats[7][2]  + list_stats[8][2]  

        # for i in list_stats: ## accumulating num of fail
        #     if((list_stats[i] ==list_stats[2]) and (flag_for_inRoute == False)):
        #         num_of_faults = num_of_faults + 1
        #         flag_for_inRoute = True
        #     elif(( list_stats[i] ==list_stats[3]) and (flag_for_inLane == False)):
        #         num_of_faults = num_of_faults + 1 
        #     elif( list_stats[i] ==list_stats[1]):
        #         pass
        #     else:
        #         num_of_faults = num_of_faults + list_stats[i][2]       

  
        if (list_stats[2][1] == "FAILURE"):
            num_of_faults = num_of_faults + 1 
        
        if (list_stats[3][1] == "FAILURE"):
            num_of_faults = num_of_faults + 1 

        if(num_of_faults >= 4):  ### checking for fail 
            self._result = 'Failed'
                
        


# #-----------------------------------------------------------------------------------------

        data = [['Evaluation Table'],
                ['Today\'s Date:', today],
                ['Start Time:', self._start_time],
                ['End Time:', end_time],
                ['Duration:', game_time], 
                ['Vehicle: ', cartype],
                ['Number of Faults:', num_of_faults],
                ['Immediate Fail:', immediate_fail]]

        # Create the table and apply style
        table = Table(data, colWidths=[340, 220])
        table.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black),
                                ('SPAN', (0,0), (1,0))]))

#         # Define other data for the table 1
# #-----------------------------------------------------------------------------------------
#         #--make variables for this--
#         #---------------------------

        data2 = [['Immediate Fail'],
                ['Collisions', list_stats[0][2]],
                ['Not Following Route', list_stats[2][2]],
                ['Crossing Redlight',list_stats[4][2] ],
                ['Sidewalk Hit(s)', list_stats[7][2]],
                ['Driving in Wrong Lane',list_stats[8][2]]
                ]

        # Create the table and apply style
        table2 = Table(data2, colWidths=[340, 220])
        table2.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (1,0))]))

#         # Define other data for the table 3
#         #---------------------------
#         #--make variables for this--
# #-----------------------------------------------------------------------------------------




        data3 = [['Faults','SUCCESS/FAILURE', 'Number of Faults'],
                ['Number of Collisions', list_stats[0][1] , list_stats[0][2] ],
                ['Not Following Route\n (Distance)', list_stats[2][1], list_stats[2][2]],
                ['Not Following Lanes\n (Distance)', list_stats[3][1], list_stats[3][2]],
                ['Crossing Redlight',list_stats[4][1], list_stats[4][2] ],
                ['Crossing Stop Sign',list_stats[5][1], list_stats[5][2]],
                ['Speed Limit Exceeded', list_stats[6][1] , list_stats[6][2]],
                ['Sidewalk Hit(s)', list_stats[7][1] , list_stats[7][2]],
                ['Driving in Wrong Lane',list_stats[8][1], list_stats[8][2]]
                ]

#         # Create the table and apply style
        table3 = Table(data3, colWidths=[120, 220])
        table3.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (1,0))
                                
                                ]))

#         # Define other data for the table 1
#         #---------------------------
#         #--make variables for this--
# #-----------------------------------------------------------------------------------------
        
# #-----------------------------------------------------------------------------------------

        data4 = [['Feedback Summary'],
                ['Faults', "SUCCESS/FAILURE", 'Advice Given'],
                ['Number of Collisions', list_stats[0][1] , self.collision_advice ],
                ['Not Following Route\n (Distance)', list_stats[2][1], self.instruction_advice],
                ['Not Following Lanes\n (Distance)', list_stats[3][1], self.in_Lane_advice],
                ['Crossing Redlight',list_stats[4][1], self.redlight_advice ],
                ['Crossing Stop Sign',list_stats[5][1], self.stop_sign_advice],
                ['Speed Limit Exceeded', list_stats[6][1] , self.speed_limit_advice],
                ['Sidewalk Hit(s)', list_stats[7][1] , self.sidewalk_advice],
                ['Driving in Wrong Lane',list_stats[8][1], self.wrong_lane_advice]
                ]
        

        # Create the table and apply style
        table4 = Table(data4, colWidths=[120, 220])
        table4.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black), 
                                ('SPAN', (0,0), (-1,0)),
                                ('VALIGN', (0,0), (-1,-1), 'MIDDLE')
                                
                                ]))


        data5 = [['Result', self._result]]

        # Create the table and apply style
        table5 = Table(data5, colWidths=[340, 220])
        table5.setStyle(TableStyle([('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
                                ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
                                ('TEXTCOLOR', (0,0), (-1,0), colors.black),
                                ('TEXTCOLOR', (0,1), (-1,-1), colors.black),
                                ('ALIGN', (0,0), (-1,-1), 'CENTER'),
                                ('FONTNAME', (0,0), (-1,0), 'Helvetica-Bold'),
                                ('FONTSIZE', (0,0), (-1,0), 12),
                                ('BOTTOMPADDING', (0,0), (-1,0), 12),
                                ('GRID', (0,0), (-1,-1), 1, colors.black)]
                                ))

#         # Create the PDF document and add the table
        doc = SimpleDocTemplate(r"C:\Users\b00083281\Desktop\Reports\Driving_Evaluations.pdf", pagesize=letter)
        #doc.build([table,table2, table3, PageBreak(), table5])
        doc.build([table, table2, table3, PageBreak(), table4, table5]) #table 4 is not initialized yet

        print('PDF report generated successfully!')