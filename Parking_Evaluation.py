from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle
from reportlab.lib import colors
import datetime
import time

# Get today's date, start time, and end time from the OS system
today = datetime.date.today().strftime('%Y-%m-%d')
start_time = time.strftime('%H:%M:%S')
end_time = time.strftime('%H:%M:%S')

# Calculate duration as the difference between end time and start time
start_time_dt = datetime.datetime.strptime(start_time, '%H:%M:%S')
end_time_dt = datetime.datetime.strptime(end_time, '%H:%M:%S')
duration_td = end_time_dt - start_time_dt
duration = str(duration_td)
cartype = "Dodge Charger"

# Define other data for the table 1
num_faults = 5
immediate_fail = 'Yes'

data = [['Evaluation Table'],
        ['Today\'s Date:', today],
        ['Start Time:', start_time],
        ['End Time:', end_time],
        ['Duration:', duration], 
        ['Vehicle: ', cartype],
        ['Number of Faults:', num_faults],
        ['Immediate Fail:', immediate_fail]]

# Create the table and apply style
table = Table(data, colWidths=[120, 220])
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

# Define other data for the table 1
data2 = [['Immediate Fail'],
        ['Collisions', '111'],]

# Create the table and apply style
table2 = Table(data2, colWidths=[120, 220])
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

# Define other data for the table 3
data3 = [['Number of Faults'],
        ['Over Speeding', ''],
        ['Not using blinkers', ''],
        ['Collisions', ''],
        ['Not Following Instructions', '']]

# Create the table and apply style
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
                            ('SPAN', (0,0), (1,0))]))

# Define other data for the table 1
data4 = [['Result', 'fail/pass']]

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
                            ('GRID', (0,0), (-1,-1), 1, colors.black)]
                            ))

# Create the PDF document and add the table
doc = SimpleDocTemplate("Parking_Evaluation.pdf", pagesize=letter)
doc.build([table, table2, table3, table4])

print('PDF report generated successfully!')


