import pyrebase

config = {
    "apiKey": "AIzaSyBYoGN_ayC7GFWIeTSdKkh4vl022YYJEqw",
    "authDomain": "hayago-42fbf.firebaseapp.com",
    "databaseURL": "https://hayago-42fbf-default-rtdb.asia-southeast1.firebasedatabase.app",
    "projectId": "hayago-42fbf",
    "storageBucket": "hayago-42fbf.appspot.com",
    "messagingSenderId": "806345285802",
    "appId": "1:806345285802:web:872dc1e96765093dff30d3",
    "measurementId": "G-S2ZSFSY0JQ"
}

firebase = pyrebase.initialize_app(config=config)

print("______________________________________\n")
print("[1] Sign In Anonymous\n")
print("[2] Sign In With Google\n")
print("[3] Sign Up With Google\n")
print("Choose one option: ")
option = input()
auth = firebase.auth()
user = ''
try:
    if option == 1:
        user = auth.sign_in_anonymous()
    else:
        print("Enter your email (gmail): ")
        email = input()
        print("\nEnter your password (gmail): ")
        password = input()
        if option == 2:
            user = auth.sign_in_with_email_and_password()
        elif option == 3:
            user = auth.create_user_with_email_and_password()
except Exception as error:
    print("An error occured\n")
    print(error.args)

database = firebase.database()