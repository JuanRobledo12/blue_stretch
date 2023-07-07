import pandas as pd
from tkinter import *
from tkinter import ttk

class Application(Frame):
    def __init__(self):
        self.master = Tk()
        super().__init__(self.master)
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.add_element_entry = Entry(self)
        self.add_element_entry.pack(side="top")

        self.add_element_button = Button(self)
        self.add_element_button["text"] = "Add element"
        self.add_element_button["command"] = self.add_element
        self.add_element_button.pack(side="top")

        self.quit = Button(self, text="QUIT", fg="red",
                              command=self.master.destroy)
        self.quit.pack(side="bottom")

        # Creating the table
        self.tree = ttk.Treeview(self.master)
        self.tree["columns"]=("Item")
        self.tree.column("#0", width=0, stretch=NO)
        self.tree.column("Item", width=100)
        self.tree.heading("Item", text="Item", anchor='w')
        self.tree.pack(side="top", fill="both", expand=1)
        
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


#Test
window = Application()
window.run()