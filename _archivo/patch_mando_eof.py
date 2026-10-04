def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Make sure we break on EOFError
    old = """    except KeyboardInterrupt:
        print("\\nSaliendo...")
        break
    except Exception as e:"""
    
    new = """    except (KeyboardInterrupt, EOFError):
        print("\\nSaliendo...")
        break
    except Exception as e:"""
    
    content = content.replace(old, new)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("mando.py")
