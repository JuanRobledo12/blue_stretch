import pandas as pd
import customtkinter as ctk
from tkinter import ttk

class Application(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.geometry("950x400")  # Set the window size
        self.title("Item Tracker")  # Set the window title
        self.create_widgets()

    def create_widgets(self):

        self.configure(bg="grey17")  # configure the window color

        title_label = ctk.CTkLabel(self, text="Currently Tracked Items GUI", font=("Arial", 18))
        title_label.place(relx=0.5, rely=0.1, anchor='center')

        self.add_element_entry = ctk.CTkEntry(self)
        self.add_element_entry.place(relx=0.75, rely=0.3, anchor='center')

        self.add_element_button = ctk.CTkButton(self, text="Add element", text_color='black', border_color='black', command=self.add_element)
        self.add_element_button.place(relx=0.75, rely=0.4, anchor='center')
        
        self.remove_element_button = ctk.CTkButton(self, text="Remove element", text_color='black', border_color='black', command=self.remove_element)
        self.remove_element_button.place(relx=0.75, rely=0.5, anchor='center')

        self.quit = ctk.CTkButton(self, text="QUIT", fg_color="orange red", text_color='black', border_color='black', hover_color='OrangeRed2', command=self.destroy)
        self.quit.place(relx=0.75, rely=0.6, anchor='center')

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

    def remove_element(self):
        selected_items = self.tree.selection()
        for item in selected_items:
            item_text = self.tree.item(item)['values'][0]
            df = pd.read_csv('./stretch_misc_files/items.csv')
            df = df[df['Item'] != item_text]
            df.to_csv('./stretch_misc_files/items.csv', index=False)
        self.update_table()

    def update_table(self):
        self.tree.delete(*self.tree.get_children())
        df = pd.read_csv('./stretch_misc_files/items.csv')
        for index, row in df.iloc[1:].iterrows():  # Exclude the first row
            self.tree.insert("", 'end', text=index-1, values=(row['Item'],))

    def run(self):
        self.mainloop()

#app = Application()
#app.run()
