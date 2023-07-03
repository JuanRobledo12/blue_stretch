import pandas as pd
import customtkinter as ctk
from tkinter import ttk

class Application(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.geometry("950x400")  # Set the window size
        self.create_widgets()

    def create_widgets(self):

        self.configure(bg="grey17")  # configure the window color

        self.add_element_entry = ctk.CTkEntry(self)
        self.add_element_entry.place(relx=0.75, rely=0.1, anchor='center')

        self.add_element_button = ctk.CTkButton(self, text="Add element", command=self.add_element)
        self.add_element_button.place(relx=0.75, rely=0.2, anchor='center')
    

        self.quit = ctk.CTkButton(self, text="QUIT", fg_color="red", command=self.destroy)
        self.quit.place(relx=0.75, rely=0.3, anchor='center')

        # Creating the table
        self.tree = ttk.Treeview(self)
        self.tree["columns"]=("Item")
        self.tree.column("#0", width=0, stretch='no')
        self.tree.column("Item", width=100)
        self.tree.heading("Item", text="Item", anchor='w')
        self.tree.place(relx=0.25, rely=0.5, anchor='center')
        
        self.update_table()

    def add_element(self):
        new_element = self.add_element_entry.get()
        df = pd.read_csv('./stretch_misc_files/items.csv')
        df = df.append({'Item': new_element}, ignore_index=True)
        df.to_csv('./stretch_misc_files/items.csv', index=False)
        self.update_table()

    def update_table(self):
        for i in self.tree.get_children():
            self.tree.delete(i)
        df = pd.read_csv('./stretch_misc_files/items.csv', header=None, names=['Item'])
        for index, row in df.iterrows():
            self.tree.insert("", 'end', text=index, values=(row['Item'],))

    def run(self):
        self.mainloop()

app = Application()
app.run()
